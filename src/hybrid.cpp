#include "hybrid.hpp"
#include "flow_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include "flow_planner.hpp"

namespace hybrid {
namespace {

// Corridor-first MDDs: UA_CORRIDOR=1 (default) tries, at each makespan, the corridor levels of UA_CORRIDOR_SLACKS
// (default "2,8") before the full MDD. UNSAT at a corridor level proves nothing; only UNSAT at the full level
// moves on to the next makespan. A solution from a corridor level is a valid plan at that makespan. The corridor
// is used only if the full MDD has at least UA_CORRIDOR_MIN_NODES nodes (default 100,000).
// UA_CORRIDOR=0: full MDD only.
std::vector<int> corridor_slacks() {
    std::vector<int> slacks;
    const char *on = std::getenv("UA_CORRIDOR");
    if (on != nullptr && std::string(on) == "0") return slacks;
    const char *text = std::getenv("UA_CORRIDOR_SLACKS");
    std::stringstream in(text != nullptr ? text : "2,8");
    std::string item;
    while (std::getline(in, item, ',')) {
        if (item.empty()) continue;
        const int slack = std::stoi(item);
        if (slack < 0) throw std::invalid_argument("UA_CORRIDOR_SLACKS must list nonnegative integers");
        slacks.push_back(slack);
    }
    return slacks;
}

std::uint64_t corridor_min_nodes() {
    const char *value = std::getenv("UA_CORRIDOR_MIN_NODES");
    return value == nullptr ? 100000 : std::stoull(value);
}

// The min-cost flow only shortens the unassigned agents' paths of the
// final solution; it does not change the makespan or the search. Off by default; UA_FLOW_MIN_COST=1
// restores the original behaviour (min-cost flow on the successful check).
bool use_min_cost_flow() {
    const char *value = std::getenv("UA_FLOW_MIN_COST");
    return value != nullptr && std::string(value) != "0";
}

Location native_location(const Problem &problem, int vertex) {
    const Cell cell = problem.vertex_cells.at(vertex);
    return Location(cell.row, cell.column);
}

int vertex_id(const Problem &problem, const Location &location) {
    if (location.x < 0 || location.x >= problem.rows || location.y < 0 || location.y >= problem.columns) {
        throw std::logic_error("Flow returned a reservation outside the map");
    }
    const int id = problem.cell_vertices.at(location.x).at(location.y);
    if (id < 0) throw std::logic_error("Flow returned a reservation on a blocked cell");
    return id;
}

vector<shared_ptr<TimedPath>> native_paths(const Problem &problem, const Paths &paths) {
    FLOW_PROFILE_SCOPE("assigned_path_conversion");
    vector<shared_ptr<TimedPath>> result;
    for (const auto &path : paths) {
        auto timed = std::make_shared<TimedPath>();
        for (std::size_t time = 0; time < path.size(); ++time) {
            timed->emplace_back(static_cast<int>(time), native_location(problem, path[time]));
        }
        result.push_back(std::move(timed));
    }
    return result;
}

CutClause cut_clause(const Problem &problem, const vector<RealizationConflict> &conflicts) {
    CutClause clause;
    for (const auto &conflict : conflicts) {
        clause.push_back({conflict.assigned_id, vertex_id(problem, conflict.location1), conflict.time});
        if (conflict.type == RealizationConflict::Type::Edge) {
            // NOT(move u->v at t) = NOT X(a,u,t) OR NOT X(a,v,t+1).
            // Flatten these terms into the one disjunction for the ENTIRE cut.
            clause.push_back({conflict.assigned_id, vertex_id(problem, conflict.location2), conflict.time + 1});
        }
    }
    std::sort(clause.begin(), clause.end());
    clause.erase(std::unique(clause.begin(), clause.end()), clause.end());
    return clause;
}

} // namespace

Result solve(const Problem &problem, const Options &options) {
    if (!std::isfinite(options.time_limit) || options.time_limit <= 0 || options.max_makespan < -1) {
        throw std::invalid_argument("Time limit must be positive; max makespan must be -1 or nonnegative");
    }
    Result result;
    const auto started = Clock::now();
    const auto deadline = started + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(options.time_limit));
    auto finish = [&](const std::string &status, const std::string &reason = "") {
        result.status = status;
        result.reason = reason;
        result.runtime_seconds = std::chrono::duration<double>(Clock::now() - started).count();
        return result;
    };
    const int lower_bound = shortest_path_lower_bound(problem);
    if (Clock::now() >= deadline) return finish("timeout", "Deadline during lower-bound computation");
    if (lower_bound < 0) return finish("infeasible", "Unreachable assigned goal or duplicate assigned goals");
    if (options.max_makespan >= 0 && lower_bound > options.max_makespan) {
        return finish("horizon_limit", "Shortest-path lower bound exceeds the requested horizon cap");
    }

    vector<vector<CellType>> map(problem.rows, vector<CellType>(problem.columns, BLOCKED));
    for (const auto &cell : problem.vertex_cells) map[cell.row][cell.column] = PASSABLE;
    vector<Location> unassigned_starts;
    for (int start : problem.unassigned_starts) unassigned_starts.push_back(native_location(problem, start));
    SolverStats flow_stats;
    std::unique_ptr<HybridFlowPlanner> flow;
    if (!unassigned_starts.empty()) {
        FLOW_PROFILE_SCOPE("planner_init");
        flow = std::make_unique<HybridFlowPlanner>(problem.rows, problem.columns, map,
            unassigned_starts, static_cast<int>(problem.assigned.starts.size()), false, &flow_stats,
            use_min_cost_flow());
    }
    // SAT clauses, not CBS constraints, constrain the assigned paths. In
    // particular, no positive committed edges are imposed on anonymous flow.
    const vector<shared_ptr<Constraints>> no_flow_constraints;
    if (options.trace) {
        *options.trace << "horizon,candidate,failed_prefix,cut_capacity,required_flow,realization_conflicts,clause_literals,negative_occupancies\n";
    }

    const std::vector<int> slacks = corridor_slacks();   // corridor levels tried before the full MDD
    SessionOptions session_options;
    session_options.corridor_min_nodes = corridor_min_nodes();
    // Every clause found so far (collisions and min-cut clauses) is added to the session of each new makespan
    // and corridor level, as in Sec. 5.1 of the paper.
    KeptClauses kept;
    for (int horizon = lower_bound;; ++horizon) {
        if (Clock::now() >= deadline) return finish("timeout", "Deadline before next horizon");
        result.last_horizon = horizon;
        ++result.horizons_attempted;
        // Stage timers include SAT setup/decoding; NrfStats separately times Glucose.
        FLOW_PROFILE_CONTEXT("sat", horizon);
        FLOW_PROFILE_SEGMENT(profile_sat_init, "sat_session_init");
        for (std::size_t level = 0; level <= slacks.size(); ++level) {
        session_options.corridor_slack = level < slacks.size() ? slacks[level] : -1;   // the last level is the full MDD
        NrfSession session(problem.assigned, horizon, deadline, result.nrf, &kept, session_options);
        FLOW_PROFILE_END(profile_sat_init);
        const bool corridor_level = session.corridor();
        while (true) {
            Paths assigned;
            FLOW_PROFILE_CONTEXT("sat", horizon);
            FLOW_PROFILE_SEGMENT(profile_sat_next, "sat_next");
            const SatStatus status = session.next(assigned);
            FLOW_PROFILE_END(profile_sat_next);
            if (status == SatStatus::Timeout) return finish("timeout", "NRF search interrupted or deadline exceeded");
            if (status == SatStatus::Unsat) {
                if (!corridor_level) ++result.horizons_proven_unsat;   // only the full MDD proves the horizon impossible
                break;
            }
            ++result.candidates;
            Paths unassigned;
            if (flow) {
                const auto flow_start = Clock::now();
                FLOW_PROFILE_CHECK(horizon);
                FLOW_PROFILE_SEGMENT(profile_check, "flow_check");
                const auto reserved = native_paths(problem, assigned);
                const auto flow_result = flow->find_paths(reserved, no_flow_constraints, horizon);
                FLOW_PROFILE_END(profile_check);
                result.flow_seconds += std::chrono::duration<double>(Clock::now() - flow_start).count();
                ++result.flow_checks;
                if (flow_result != HybridFlowPlanner::SUCCESS) ++result.flow_failures;
                if (Clock::now() >= deadline) return finish("timeout", "Deadline during flow computation");

                if (flow_result == HybridFlowPlanner::INFEASIBLE) {
                    FLOW_PROFILE_SCOPE("cut_clause_learning");
                    const auto clause_start = Clock::now();
                    const auto &conflicts = flow->get_minimum_conflict_cut_realization_constraints();
                    const CutClause clause = cut_clause(problem, conflicts);
                    result.last_failed_prefix = flow->active_horizon();
                    result.last_cut_capacity = flow->achieved_flow();
                    if (result.last_cut_capacity >= static_cast<int>(unassigned_starts.size())) {
                        throw std::logic_error("Flow failure did not produce a deficient cut");
                    }
                    session.forbid_cut(clause);
                    ++result.cut_clauses;
                    result.cut_events += conflicts.size();
                    result.cut_literals += clause.size();
                    result.clause_seconds += std::chrono::duration<double>(Clock::now() - clause_start).count();
                    if (options.trace) {
                        *options.trace << horizon << ',' << result.candidates << ',' << result.last_failed_prefix
                            << ',' << result.last_cut_capacity << ',' << unassigned_starts.size() << ','
                            << conflicts.size() << ',' << clause.size() << ',';
                        for (std::size_t i = 0; i < clause.size(); ++i) {
                            if (i) *options.trace << ';';
                            *options.trace << clause[i].agent << ':' << clause[i].vertex << ':' << clause[i].time;
                        }
                        *options.trace << '\n';
                        options.trace->flush();
                    }
                    continue;
                }
                if (flow_result != HybridFlowPlanner::SUCCESS) {
                    throw std::logic_error("Unexpected flow status without a deficient-cut certificate");
                }
                if (flow->agent_paths.size() != unassigned_starts.size()) {
                    throw std::logic_error("Flow returned the wrong number of paths");
                }
                for (const auto &path : flow->agent_paths) {
                    if (!path || path->empty()) throw std::logic_error("Flow returned an empty path");
                    std::vector<int> positions;
                    for (int t = 0; t <= horizon; ++t) positions.push_back(vertex_id(problem, path->get_location_at_time(t)));
                    unassigned.push_back(std::move(positions));
                }
            }
            validate_paths(problem, assigned, unassigned, horizon);
            if (Clock::now() >= deadline) return finish("timeout", "Deadline during solution validation");
            result.makespan = horizon;
            result.corridor_solved = corridor_level ? 1 : 0;
            result.assigned_paths = std::move(assigned);
            result.unassigned_paths = std::move(unassigned);
            return finish("solved");
        }
        if (!corridor_level) break;   // the full MDD (or a corridor that is the full MDD) is UNSAT: next makespan
        }   // corridor levels
        if ((options.max_makespan >= 0 && horizon >= options.max_makespan) ||
            horizon >= std::numeric_limits<int>::max() - 1) {
            return finish("horizon_limit", "No solution within the searched horizons; global infeasibility not claimed");
        }
    }
}

} // namespace hybrid
