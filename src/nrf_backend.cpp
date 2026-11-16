#include "nrf_backend.hpp"
#include <tuple>
#include <map>

#include "defs.h"
#include "core/smtcbs.h"
#include <algorithm>
#include <set>
#include <stdexcept>

namespace hybrid {
namespace {

double seconds_since(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

std::unique_ptr<boOX::sInstance> make_instance(const GraphProblem &problem) {
    if (problem.vertices <= 0 || problem.starts.size() != problem.goals.size()) {
        throw std::invalid_argument("Invalid assigned-agent graph problem");
    }
    boOX::sUndirectedGraph graph;
    graph.add_Vertices(problem.vertices);
    for (const auto &[from, to] : problem.edges) {
        if (from < 0 || to < 0 || from >= problem.vertices || to >= problem.vertices || from == to) {
            throw std::invalid_argument("Invalid graph edge");
        }
        graph.add_Edge(from, to);
    }
    const int count = static_cast<int>(problem.starts.size());
    boOX::sConfiguration starts(problem.vertices, count);
    boOX::sConfiguration goals(problem.vertices, count);
    std::set<int> start_set, goal_set;
    for (int agent = 0; agent < count; ++agent) {
        const int start = problem.starts.at(agent), goal = problem.goals.at(agent);
        if (start < 0 || goal < 0 || start >= problem.vertices || goal >= problem.vertices ||
            !start_set.insert(start).second || !goal_set.insert(goal).second) {
            throw std::invalid_argument("Invalid or duplicated assigned start/goal");
        }
        starts.place_Agent(agent + 1, start);
        goals.place_Agent(agent + 1, goal);
    }
    return std::make_unique<boOX::sInstance>(graph, starts, goals);
}

} // namespace

struct KeptClauses::Data {
    boOX::sSMTCBS::Collisions_vector collisions;          // as boOX's Context::m_trans_Collisions
    boOX::sSMTCBS::EdgeCollisions_vector edge_collisions; // as boOX's Context::m_trans_edge_Collisions
    std::vector<CutClause> cuts;                          // flow cut clauses, in learned order
};

KeptClauses::KeptClauses() : data_(std::make_unique<Data>()) {}
KeptClauses::~KeptClauses() = default;
std::size_t KeptClauses::collisions() const { return data_->collisions.size() + data_->edge_collisions.size(); }
std::size_t KeptClauses::cuts() const { return data_->cuts.size(); }

struct NrfSession::Impl {
    GraphProblem problem;
    int horizon;
    Clock::time_point deadline;
    NrfStats &stats;
    KeptClauses *kept;
    SessionOptions options;
    bool corridor_active = false;
    std::unique_ptr<boOX::sInstance> instance;
    boOX::sBoolEncoder encoder;
    std::unique_ptr<boOX::sSMTCBS> nrf;
    boOX::sSMTCBS::Context context;
    boOX::sSMTCBS::Model model;
    boOX::sInstance::MDD_vector mdd, extra_mdd;
    boOX::sInstance::InverseMDD_vector inverse_mdd;
    Glucose::Solver solver;
    Paths candidate;
    std::set<std::vector<int>> learned_cuts;
    bool infeasible = false;
    bool has_candidate = false;

    Impl(const GraphProblem &input, int time_horizon, Clock::time_point end, NrfStats &counters,
         KeptClauses *kept_clauses, const SessionOptions &session_options)
        : problem(input), horizon(time_horizon), deadline(end), stats(counters), kept(kept_clauses),
          options(session_options) {
        if (horizon < 0) throw std::invalid_argument("Negative time horizon");
        const auto start = Clock::now();
        instance = make_instance(problem);
        solver.verbosity = 0;
        solver.s_Glucose_timeout = -1;
        if (!problem.starts.empty()) {
            nrf = std::make_unique<boOX::sSMTCBS>(&encoder, instance.get());
            instance->construct_MakespanRotationMDD(horizon, mdd, extra_mdd);
            if (options.corridor_slack >= 0 && mdd_nodes() >= options.corridor_min_nodes) {
                corridor_active = true;
                ++stats.corridor_sessions;
                restrict_to_corridor();
            }
            instance->construct_InverseMDD(mdd, inverse_mdd);
            for (std::size_t agent = 1; agent < mdd.size(); ++agent) {
                for (const auto &layer : mdd[agent]) {
                    if (layer.empty()) infeasible = true;
                }
            }
            if (!infeasible) {
                // Exactly boOX's InverseOmitted (NRF-SAT) encoding. Empty
                // extra_mdd layers mean no FUEL+/sum-of-costs bound is imposed.
                const int next_variable = nrf->build_RotationSmallModelVariablesInverse(
                    &solver, context, *instance, mdd, extra_mdd, inverse_mdd, horizon, 0, model);
                encoder.set_LastVariableID(next_variable);
                if (kept) load_kept_collisions();   // the variables exist now; boOX adds them below
                nrf->build_RotationTinyModelConstraintsInverse(
                    &solver, context, *instance, mdd, extra_mdd, inverse_mdd, horizon, 0, model);
                if (kept) load_kept_cuts();
            }
        }
        stats.encoding_seconds += seconds_since(start);
    }

    std::uint64_t mdd_nodes() const {
        std::uint64_t nodes = 0;
        for (std::size_t agent = 1; agent < mdd.size(); ++agent) {
            for (const auto &layer : mdd[agent]) nodes += layer.size();
        }
        return nodes;
    }

    // Corridor: keep in layer t of agent a only the cells on some path of length <= d_a + slack from the start
    // to the goal (ds(v) + dg(v) <= d_a + slack). Each kept cell still has a successor and a predecessor inside
    // the corridor (one step along a shortest path), and no layer is empty (the shortest paths stay).
    void restrict_to_corridor() {
        const auto &from_start = instance->m_environment.get_SourceShortestPaths();
        const auto &to_goal = instance->m_environment.get_GoalShortestPaths();
        for (std::size_t agent = 1; agent < mdd.size(); ++agent) {
            const int start = problem.starts.at(agent - 1), goal = problem.goals.at(agent - 1);
            const int limit = from_start[start][goal] + options.corridor_slack;
            for (auto &layer : mdd[agent]) {
                layer.erase(std::remove_if(layer.begin(), layer.end(), [&](int v) {
                    return from_start[start][v] + to_goal[goal][v] > limit;
                }), layer.end());
            }
        }
    }

    bool expired() const { return Clock::now() >= deadline; }

    void check_individual_paths(const boOX::sSMTCBS::AgentPaths_vector &paths) const {
        if (paths.size() != problem.starts.size() + 1) {
            throw std::runtime_error("NRF returned the wrong number of assigned paths");
        }
        for (std::size_t a = 0; a < problem.starts.size(); ++a) {
            const auto &path = paths.at(a + 1);
            if (path.size() != static_cast<std::size_t>(horizon) + 1 ||
                path.front() != problem.starts[a] || path.back() != problem.goals[a]) {
                throw std::runtime_error("NRF path extraction violated a boundary condition");
            }
            for (int t = 0; t <= horizon; ++t) {
                if (path[t] < 0 || path[t] >= problem.vertices ||
                    (t > 0 && path[t] != path[t - 1] &&
                     !instance->m_environment.is_Adjacent(path[t - 1], path[t]))) {
                    throw std::runtime_error("NRF path extraction produced an invalid move");
                }
            }
        }
    }

    SatStatus next(Paths &paths) {
        paths.clear();
        has_candidate = false;
        if (expired()) return SatStatus::Timeout;
        if (infeasible) return SatStatus::Unsat;
        if (problem.starts.empty()) {
            candidate.clear();
            has_candidate = true;
            return SatStatus::Candidate;
        }
        while (!expired()) {
            const auto sat_start = Clock::now();
            solver.s_Glucose_timeout = std::chrono::duration<double>(deadline - sat_start).count();
            Glucose::vec<Glucose::Lit> assumptions;
            ++stats.sat_calls;
            // The bool-returning boOX convenience functions conflate UNSAT
            // and interruption. Keep Glucose's three-valued result instead.
            const Glucose::lbool status = solver.solveLimited(assumptions);
            stats.sat_seconds += seconds_since(sat_start);
            if (expired() || status == l_Undef) return SatStatus::Timeout;
            if (status == l_False) {
                infeasible = true;
                return SatStatus::Unsat;
            }
            if (status != l_True) throw std::runtime_error("Unexpected SAT status");

            const auto decode_start = Clock::now();
            boOX::sSMTCBS::AgentPaths_vector decoded;
            nrf->decode_RotationTreeModel(&solver, *instance, mdd, model, decoded);
            check_individual_paths(decoded);
            boOX::sSMTCBS::Collisions_vector vertices;
            boOX::sSMTCBS::EdgeCollisions_vector edges;
            const auto check = nrf->check_NonconflictingRotation(*instance, decoded, vertices, edges);
            if (check.first >= 0) {
                candidate.assign(decoded.begin() + 1, decoded.end());
                paths = candidate;
                has_candidate = true;
                stats.decode_check_seconds += seconds_since(decode_start);
                return expired() ? SatStatus::Timeout : SatStatus::Candidate;
            }
            if (vertices.empty() && edges.empty()) {
                throw std::runtime_error("Assigned collision checker failed without an explanation");
            }
            ++stats.assigned_refinements;
            stats.assigned_collision_events += vertices.size() + edges.size();
            nrf->refine_RotationSmallModelCollisionsInverse(
                &solver, vertices, edges, *instance, mdd, extra_mdd, inverse_mdd,
                horizon, 0, model);
            if (kept) {   // as boOX's find_Next...: every new collision also goes to the kept list
                auto &data = kept->data();
                data.collisions.insert(data.collisions.end(), vertices.begin(), vertices.end());
                data.edge_collisions.insert(data.edge_collisions.end(), edges.begin(), edges.end());
            }
            stats.decode_check_seconds += seconds_since(decode_start);
        }
        return SatStatus::Timeout;
    }

    int occupancy_id(int agent, int vertex, int time) const {
        const auto &layer = inverse_mdd.at(agent + 1).at(time);
        const auto found = layer.find(vertex);
        if (found == layer.end()) return 0;
        return model.m_vertex_occupancy.at(agent + 1).at(time).at(found->second);
    }

    bool in_mdd(int boox_agent, int level, int vertex) const {
        return level >= 0 && level <= horizon && occupancy_id(boox_agent - 1, vertex, level) > 0;
    }

    // Keep, step 1 (before boOX builds the formula): boOX's encoder adds every collision of `context` itself,
    // exactly as in boOX's own makespan loop. A collision whose variable is missing is satisfied and skipped.
    void load_kept_collisions() {
        for (const auto &collision : kept->data().collisions) {
            if (in_mdd(collision.m_agent_A_id, collision.m_level_A, collision.m_vertex_A_id) &&
                in_mdd(collision.m_agent_B_id, collision.m_level_B, collision.m_vertex_B_id)) {
                context.m_trans_Collisions.push_back(collision);
            }
        }
        // boOX does use edge collisions (a 4-literal mutex on both agents' occupancies at level and level + 1).
        // In a corridor MDD one of the four variables can be missing, which would crash boOX: filter on all four.
        for (const auto &edge : kept->data().edge_collisions) {
            if (in_mdd(edge.m_agent_A_id, edge.m_level_A, edge.m_edge_A_u_id) &&
                in_mdd(edge.m_agent_A_id, edge.m_level_A + 1, edge.m_edge_A_v_id) &&
                in_mdd(edge.m_agent_B_id, edge.m_level_B, edge.m_edge_B_u_id) &&
                in_mdd(edge.m_agent_B_id, edge.m_level_B + 1, edge.m_edge_B_v_id)) {
                context.m_trans_edge_Collisions.push_back(edge);
            }
        }
        stats.kept_collisions += context.m_trans_Collisions.size();
    }

    // Keep, step 2 (after the formula is built): the flow cut clauses of the earlier makespans.
    void load_kept_cuts() {
        for (const auto &clause : kept->data().cuts) {
            std::vector<int> literals;
            bool satisfied = false;
            for (const auto &event : clause) {
                const int id = event.time >= 0 && event.time <= horizon
                    ? occupancy_id(event.agent, event.vertex, event.time) : 0;
                if (id <= 0) { satisfied = true; break; }
                literals.push_back(-id);
            }
            if (satisfied) continue;
            std::sort(literals.begin(), literals.end());
            literals.erase(std::unique(literals.begin(), literals.end()), literals.end());
            if (!learned_cuts.insert(literals).second) continue;
            ++stats.kept_cuts;
            if (literals.empty()) infeasible = true;   // a deficient cut without blockers holds at every makespan
            else encoder.cast_Clause(&solver, literals);
        }
    }

    void forbid_cut(const CutClause &clause) {
        if (!has_candidate) throw std::logic_error("A flow clause requires an assigned candidate");
        std::vector<int> literals;
        for (const auto &event : clause) {
            if (event.agent < 0 || event.agent >= static_cast<int>(candidate.size()) ||
                event.time < 0 || event.time > horizon ||
                candidate.at(event.agent).at(event.time) != event.vertex) {
                throw std::logic_error("Cut event does not occur on the extracted assigned paths");
            }
            const auto &layer = inverse_mdd.at(event.agent + 1).at(event.time);
            const auto found = layer.find(event.vertex);
            if (found == layer.end()) throw std::logic_error("Cut event is missing from the MDD");
            const int id = model.m_vertex_occupancy.at(event.agent + 1).at(event.time).at(found->second);
            if (id <= 0 || id > solver.model.size() || solver.model[id - 1] != l_True) {
                throw std::logic_error("New cut clause does not reject the current SAT model");
            }
            literals.push_back(-id);
        }
        std::sort(literals.begin(), literals.end());
        literals.erase(std::unique(literals.begin(), literals.end()), literals.end());
        if (!learned_cuts.insert(literals).second) {
            throw std::logic_error("A previously learned cut was violated again");
        }
        if (literals.empty()) {
            // A deficient cut without assigned blockers cannot be repaired at T.
            infeasible = true;
        } else {
            encoder.cast_Clause(&solver, literals);
        }
        if (kept) kept->data().cuts.push_back(clause);
        has_candidate = false;
    }
};

NrfSession::NrfSession(const GraphProblem &problem, int horizon, Clock::time_point deadline,
                       NrfStats &stats, KeptClauses *kept, const SessionOptions &options)
    : impl_(std::make_unique<Impl>(problem, horizon, deadline, stats, kept, options)) {}
NrfSession::~NrfSession() = default;
SatStatus NrfSession::next(Paths &paths) { return impl_->next(paths); }
bool NrfSession::corridor() const { return impl_->corridor_active; }
void NrfSession::forbid_cut(const CutClause &clause) { impl_->forbid_cut(clause); }

} // namespace hybrid
