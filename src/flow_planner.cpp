#include "flow_planner.hpp"
#include "flow_profile.hpp"

#if STRIP_OWN_WITH_ORTOOLS
#include "ortools/graph/max_flow.h"
#endif

#include <algorithm>
#include <chrono>
#include <cstdio>
namespace {
struct DebugCounters {
    long long cancels = 0, augments = 0, failed_bfs = 0, bfs_nodes = 0, repairs = 0, fresh = 0, minrc = 0;
    long long ortools_builds = 0, ortools_solvers = 0, ortools_solves = 0;
    double fresh_s = 0;
    double repair_s = 0, minrc_s = 0, rest_cut_s = 0, solve_s = 0, update_s = 0;
    long long own_calls = 0, phases = 0, phase_nodes = 0, prefix_nodes = 0, area_a = 0, area_b = 0;
    ~DebugCounters() {
        const char *v = std::getenv("UA_FLOW_COUNTS");
        if (v && std::string(v) != "0" && own_calls > 0) {
            std::fprintf(stderr, "MINRC per call: steps(passes)=%.1f, points visited per step=%.0f, network points=%.0f, "
                         "area A=%.0f, area B=%.0f, middle strip=%.0f\n", double(phases) / own_calls,
                         double(phase_nodes) / std::max(1LL, phases), double(prefix_nodes) / own_calls,
                         double(area_a) / own_calls, double(area_b) / own_calls,
                         double(prefix_nodes - area_a - area_b) / own_calls);
        }
        if (v && std::string(v) != "0") {
            std::fprintf(stderr, "COUNTS repairs=%lld fresh=%lld cancels=%lld augments=%lld failed_bfs=%lld bfs_nodes=%lld "
                         "repair_s=%.3f fresh_s=%.3f minrc_calls=%lld minrc_s=%.3f (solve %.3f, cut-listing %.3f) "
                         "ortools_graph_builds=%lld ortools_solvers=%lld ortools_solves=%lld\n", repairs, fresh, cancels, augments, failed_bfs,
                         bfs_nodes, repair_s, fresh_s, minrc, minrc_s, solve_s, update_s, ortools_builds, ortools_solvers, ortools_solves);
        }
    }
} debug_counters;
using DebugClock = std::chrono::steady_clock;
}
#include <cstdlib>
#include <limits>
#include <queue>
#include <string>

HybridFlowPlanner::HybridFlowPlanner(int map_rows,
                                           int map_cols,
                                           const vector<vector<CellType>> &map,
                                           vector<Location> &agent_start_locations,
                                           int num_assigned_agents,
                                           bool verbose,
                                           SolverStats *stats,
                                           bool use_min_cost_flow) :
        map_rows(map_rows),
        map_cols(map_cols),
        map(map),
        num_of_agents(static_cast<int>(agent_start_locations.size())),
        num_assigned_agents(num_assigned_agents),
        agent_start_locations(agent_start_locations),
        verbose(verbose),
        stats(stats),
        use_min_cost_flow(use_min_cost_flow),
        source_node(EarliestFailMinRCDirectFlowNode::SOURCE, -1),
        source_idx(get_node_idx(source_node)),
        sink_node(EarliestFailMinRCDirectFlowNode::SINK, -1),
        sink_idx(get_node_idx(sink_node)),
        // No prefix is active before the first find_paths() call.
        current_makespan(-1),
        max_makespan(0),
        num_edges_before_sink(0),
        active_node_count(0),
        active_non_sink_arc_count(0),
        maximum_flow(0),
        optimal_cost(0),
        agent_paths(num_of_agents, shared_ptr<TimedPath>()),
        realization_conflict(nullptr) {
    if (const char *value = std::getenv("UA_FLOW_REUSE_TSTAR")) {
        reuse_earliest_failing_probe = std::string(value) != "0";
    }
    if (const char *value = std::getenv("UA_FLOW_INCREMENTAL")) {
        use_incremental_flow = std::string(value) != "0";
    }
    if (const char *value = std::getenv("UA_FLOW_OWN_MINRC")) {
        use_own_minrc = std::string(value) != "0";
    }
    if (const char *value = std::getenv("UA_FLOW_AUGMENT")) {
        use_dfs_augment = std::string(value) != "bfs";
    }
    if (const char *value = std::getenv("UA_FLOW_INCREMENTAL_VERIFY")) {
        incremental_verify = std::string(value) != "0";
    }
    if (const char *value = std::getenv("UA_FLOW_WARM_TSTAR")) {
        warm_start_tstar = std::string(value) != "0";
    }
    if (const char *value = std::getenv("UA_FLOW_EARLIEST_FAIL")) {
        use_earliest_failing_prefix = std::string(value) != "0";
    }
    if (const char *value = std::getenv("UA_FLOW_MIN_RC")) {
        use_minimum_conflict_cut = std::string(value) != "0";
    }
#if !STRIP_OWN_WITH_ORTOOLS
    if (use_min_cost_flow || !use_incremental_flow || !use_own_minrc || incremental_verify) {
        throw std::invalid_argument(
                "This OR-Tools-free build requires min-cost=0, incremental=1, own-minrc=1, verify=0.");
    }
#endif
    reached_locations.reserve(map_rows * map_cols);
    map_edges.reserve(map_rows * map_cols * 2 - map_rows - map_cols);
    new_reached_locations.reserve(num_of_agents * 4);

    for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
        NodeIndex in_idx = get_node_idx(
                EarliestFailMinRCDirectFlowNode(EarliestFailMinRCDirectFlowNode::IN, 0, agent_start_locations.at(agent_id)));
        NodeIndex out_idx = get_node_idx(
                EarliestFailMinRCDirectFlowNode(EarliestFailMinRCDirectFlowNode::OUT, 0, agent_start_locations.at(agent_id)));
        add_arc(source_idx, in_idx);
        add_arc(in_idx, out_idx);
        reached_locations.push_back(agent_start_locations.at(agent_id));
        new_reached_locations.insert(agent_start_locations.at(agent_id));
    }

    active_node_count = static_cast<NodeIndex>(idx_to_node.size());
    active_non_sink_arc_count = static_cast<ArcIndex>(num_edges_before_sink);
    makespan_to_node_count.assign(1, active_node_count);
    makespan_to_non_sink_arc_count.assign(1, active_non_sink_arc_count);
}

HybridFlowPlanner::NodeIndex
HybridFlowPlanner::find_node(EarliestFailMinRCDirectFlowNode::Type type,
                             int time,
                             const Location &location) const {
    if (time < 0 || time >= static_cast<int>(layer_nodes.size()) ||
        location.x < 0 || location.x >= map_rows || location.y < 0 || location.y >= map_cols) {
        return -1;
    }
    return layer_nodes[time][node_slot(type, location)];
}

HybridFlowPlanner::NodeIndex
HybridFlowPlanner::at_node(EarliestFailMinRCDirectFlowNode::Type type,
                           int time,
                           const Location &location) const {
    const NodeIndex node = find_node(type, time, location);
    if (node < 0) {
        throw std::out_of_range("Direct-flow node does not exist.");
    }
    return node;
}

HybridFlowPlanner::NodeIndex
HybridFlowPlanner::get_node_idx(const EarliestFailMinRCDirectFlowNode &node) {
    if (node.type == EarliestFailMinRCDirectFlowNode::SOURCE ||
        node.type == EarliestFailMinRCDirectFlowNode::SINK) {
        // The source and the sink are created exactly once, in the constructor.
        const auto idx = static_cast<NodeIndex>(idx_to_node.size());
        idx_to_node.push_back(node);
        out_arcs.emplace_back();
        return idx;
    }
    if (node.time >= static_cast<int>(layer_nodes.size())) {
        layer_nodes.resize(node.time + 1, vector<NodeIndex>(2 * map_rows * map_cols, -1));
    }
    NodeIndex &slot = layer_nodes[node.time][node_slot(node.type, node.location)];
    if (slot >= 0) {
        return slot;
    }
    slot = static_cast<NodeIndex>(idx_to_node.size());
    idx_to_node.push_back(node);
    out_arcs.emplace_back();
    return slot;
}

HybridFlowPlanner::ArcIndex
HybridFlowPlanner::get_arc_idx(NodeIndex tail, NodeIndex head) const {
    for (ArcIndex arc : out_arcs.at(tail)) {
        if (arc_head[arc] == head) {
            return arc;
        }
    }
    throw std::out_of_range("Direct-flow arc does not exist.");
}

bool HybridFlowPlanner::has_arc(NodeIndex tail, NodeIndex head) const {
    if (tail < 0 || tail >= static_cast<NodeIndex>(out_arcs.size())) {
        return false;
    }
    for (ArcIndex arc : out_arcs[tail]) {
        if (arc_head[arc] == head) {
            return true;
        }
    }
    return false;
}

void HybridFlowPlanner::add_arc(NodeIndex tail, NodeIndex head, CostValue cost) {
    assert(!has_arc(tail, head));
    const auto idx = static_cast<ArcIndex>(arc_tail.size());
    out_arcs[tail].push_back(idx);
    arc_tail.push_back(tail);
    arc_head.push_back(head);
    arc_cost.push_back(cost);
    arc_cap.push_back(CAP);
    num_edges_before_sink++;
}

void HybridFlowPlanner::update_map_edges() {
    if (reached_all) {
        return;
    }

    reached_all = true;
    unordered_set<Location> next_reached_locations(new_reached_locations.size() * 4);
    unordered_set<Location> processed_new_locations(new_reached_locations.size());

    for (const Location &location : new_reached_locations) {
        for (int direction = 0; direction < 4; ++direction) {
            Location next(location.x + dx[direction], location.y + dy[direction]);
            if (next.x < 0 || next.x >= map_rows || next.y < 0 || next.y >= map_cols ||
                map.at(next.x).at(next.y) == CellType::BLOCKED ||
                map.at(next.x).at(next.y) == CellType::STATIC_OBSTACLE) {
                continue;
            }

            const std::pair<Location, Location> edge =
                    location < next ? std::make_pair(location, next) : std::make_pair(next, location);

            if (recently_reached_locations.count(next) || processed_new_locations.count(next)) {
                assert(std::find(map_edges.begin(), map_edges.end(), edge) != map_edges.end());
                continue;
            }
            if (next_reached_locations.count(next) || new_reached_locations.count(next)) {
                assert(std::find(map_edges.begin(), map_edges.end(), edge) == map_edges.end());
                map_edges.push_back(edge);
                continue;
            }

            assert(std::find(map_edges.begin(), map_edges.end(), edge) == map_edges.end());
            map_edges.push_back(edge);
            auto [_, inserted] = next_reached_locations.emplace(next);
            assert(inserted);
            reached_all = false;
            assert(std::find(reached_locations.begin(), reached_locations.end(), next) ==
                   reached_locations.end());
            reached_locations.push_back(next);
        }
        processed_new_locations.emplace(location);
    }

    if (reached_all) {
        recently_reached_locations.clear();
        new_reached_locations.clear();
        return;
    }

    recently_reached_locations = std::move(new_reached_locations);
    new_reached_locations = std::move(next_reached_locations);
}

void HybridFlowPlanner::prepare_horizon(int makespan) {
    // Layers are created exactly as in the original planner (same node and arc indices). The OR-Tools
    // graph is built once per horizon; probes and the Min-RC stage only change capacities.
    FLOW_PROFILE_SEGMENT(graph_profile, "graph_reconfigure");
    if (makespan > max_makespan) {
        FLOW_PROFILE_NEXT(graph_profile, graph_horizon < 0 ? "graph_initial_layers" : "graph_extension_layers");
        FLOW_PROFILE_COUNT("layer_growth_calls", 1);
        FLOW_PROFILE_COUNT("layers_added", makespan - max_makespan);
    }
    for (int time = max_makespan + 1; time <= makespan; ++time) {
        // Waiting: v_out(t-1) -> v_in(t), with zero movement cost.
        for (const Location &location : reached_locations) {
            NodeIndex previous_out = at_node(EarliestFailMinRCDirectFlowNode::OUT, time - 1, location);
            NodeIndex current_in = get_node_idx(
                    EarliestFailMinRCDirectFlowNode(EarliestFailMinRCDirectFlowNode::IN, time, location));
            add_arc(previous_out, current_in, NO_COST);
        }

        // Movement: direct directed arcs replace the old EDGE_IN/EDGE_OUT gadget.
        update_map_edges();
        for (const auto &[first, second] : map_edges) {
            const NodeIndex first_out = find_node(EarliestFailMinRCDirectFlowNode::OUT, time - 1, first);
            const NodeIndex second_out = find_node(EarliestFailMinRCDirectFlowNode::OUT, time - 1, second);
            assert(first_out >= 0 || second_out >= 0);

            NodeIndex first_in = get_node_idx(EarliestFailMinRCDirectFlowNode(EarliestFailMinRCDirectFlowNode::IN, time, first));
            NodeIndex second_in = get_node_idx(EarliestFailMinRCDirectFlowNode(EarliestFailMinRCDirectFlowNode::IN, time, second));
            if (first_out >= 0) {
                add_arc(first_out, second_in, UNIT_COST);
            }
            if (second_out >= 0) {
                add_arc(second_out, first_in, UNIT_COST);
            }
        }

        // Vertex capacity at the new layer.
        for (const Location &location : reached_locations) {
            NodeIndex current_in = at_node(EarliestFailMinRCDirectFlowNode::IN, time, location);
            NodeIndex current_out = get_node_idx(
                    EarliestFailMinRCDirectFlowNode(EarliestFailMinRCDirectFlowNode::OUT, time, location));
            add_arc(current_in, current_out, NO_COST);
        }

        if (static_cast<int>(makespan_to_node_count.size()) <= time) {
            makespan_to_node_count.resize(time + 1, 0);
            makespan_to_non_sink_arc_count.resize(time + 1, 0);
        }
        makespan_to_node_count[time] = static_cast<NodeIndex>(idx_to_node.size());
        makespan_to_non_sink_arc_count[time] = static_cast<ArcIndex>(num_edges_before_sink);
    }
    max_makespan = std::max(max_makespan, makespan);

    if (makespan > graph_horizon) {
        FLOW_PROFILE_NEXT(graph_profile, "sink_layout");
        build_flow_graph(makespan);
    }
}

void HybridFlowPlanner::build_flow_graph(int makespan) {
#if STRIP_OWN_WITH_ORTOOLS
    // Solvers point into the graph: release them before the graph.
    for (auto &solver : ordinary_flow) {
        solver.reset();
    }
    weighted_flow.reset();
#endif

    graph_horizon = makespan;
    graph_non_sink_arc_count = makespan_to_non_sink_arc_count.at(makespan);
    const NodeIndex node_count = makespan_to_node_count.at(makespan);
    layer_sink_base = node_count;                       // L_t = node_count + t, t = 0..T
    total_source = node_count + makespan + 1;
    total_sink = total_source + 1;
    graph_node_count = total_sink + 1;

    // Arcs OUT(t, v) -> L_t of every layer t, for each reached cell v, in the order the original planner
    // adds its sink arcs when t is the active horizon.
    sink_arc_begin.assign(makespan + 2, 0);
    sink_arc_tails.clear();
    for (int time = 0; time <= makespan; ++time) {
        sink_arc_begin[time] = graph_non_sink_arc_count + static_cast<ArcIndex>(sink_arc_tails.size());
        const NodeIndex layer_node_count = makespan_to_node_count.at(time);
        for (const Location &location : reached_locations) {
            const NodeIndex out = find_node(EarliestFailMinRCDirectFlowNode::OUT, time, location);
            if (out >= 0 && out < layer_node_count) {
                sink_arc_tails.push_back(out);
            }
        }
    }
    sink_arc_begin[makespan + 1] = graph_non_sink_arc_count + static_cast<ArcIndex>(sink_arc_tails.size());
    layer_sink_arc_base = sink_arc_begin[makespan + 1];
    total_source_arc = layer_sink_arc_base + makespan + 1;
    total_sink_arc = total_source_arc + 1;
    const ArcIndex num_arcs = total_sink_arc + 1;

    (void) num_arcs;
    // This version: the OR-Tools copy of the network is built only when an OR-Tools solver is needed
    // (verify mode, UA_FLOW_INCREMENTAL=0, UA_FLOW_OWN_MINRC=0 or the min-cost option).
#if STRIP_OWN_WITH_ORTOOLS
    flow_graph.reset();
    arc_permutation.clear();
    ortools_graph_ready = false;
    for (int solver = 0; solver < 2; ++solver) {
        solver_open_layer[solver] = -1;
        solver_check[solver] = -1;
        solver_blocked[solver].clear();
    }
#endif
}

#if STRIP_OWN_WITH_ORTOOLS
void HybridFlowPlanner::ensure_ortools_graph() {
    if (ortools_graph_ready) {
        return;
    }
    for (auto &solver : ordinary_flow) {
        solver.reset();
    }
    weighted_flow.reset();
    const int makespan = graph_horizon;
    const ArcIndex num_arcs = total_sink_arc + 1;
    flow_graph = std::make_unique<Graph>(graph_node_count, num_arcs);
    for (ArcIndex arc = 0; arc < graph_non_sink_arc_count; ++arc) {
        flow_graph->AddArc(arc_tail[arc], arc_head[arc]);
    }
    for (int time = 0; time <= makespan; ++time) {
        for (ArcIndex arc = sink_arc_begin[time]; arc < sink_arc_begin[time + 1]; ++arc) {
            flow_graph->AddArc(sink_arc_tails[arc - graph_non_sink_arc_count], layer_sink_base + time);
        }
    }
    for (int time = 0; time <= makespan; ++time) {
        flow_graph->AddArc(layer_sink_base + time, sink_idx);
    }
    flow_graph->AddArc(total_source, source_idx);
    flow_graph->AddArc(sink_idx, total_sink);
    flow_graph->Build(&arc_permutation);
    for (int solver = 0; solver < 2; ++solver) {
        solver_open_layer[solver] = -1;
        solver_check[solver] = -1;
        solver_blocked[solver].clear();
    }
    ortools_graph_ready = true;
    ++debug_counters.ortools_builds;
}

HybridFlowPlanner::MaxFlow &HybridFlowPlanner::ordinary_solver(int solver) {
    ensure_ortools_graph();
    if (!ordinary_flow[solver]) {
        ++debug_counters.ortools_solvers;
        // Fresh solver: every non-sink arc and every OUT -> L_t arc open, every layer closed (L_t -> sink
        // at 0), total arcs at the agent count. Reservations and the open layer are set in solve_prefix().
        ordinary_flow[solver] = std::make_unique<MaxFlow>(flow_graph.get(), total_source, total_sink);
        MaxFlow &flow = *ordinary_flow[solver];
        for (ArcIndex arc = 0; arc < layer_sink_arc_base; ++arc) {
            flow.SetArcCapacity(permuted_arc(arc), CAP);
        }
        flow.SetArcCapacity(permuted_arc(total_source_arc), num_of_agents);
        flow.SetArcCapacity(permuted_arc(total_sink_arc), num_of_agents);
        solver_open_layer[solver] = -1;
        solver_check[solver] = -1;
        solver_blocked[solver].clear();
    }
    return *ordinary_flow[solver];
}

HybridFlowPlanner::MaxFlow &HybridFlowPlanner::weighted_solver() {
    ensure_ortools_graph();
    if (!weighted_flow) {
        ++debug_counters.ortools_solvers;
        weighted_flow = std::make_unique<MaxFlow>(flow_graph.get(), total_source, total_sink);
    }
    return *weighted_flow;
}
#endif

void HybridFlowPlanner::set_active_prefix(int makespan) {
    current_makespan = makespan;
    active_node_count = makespan_to_node_count.at(makespan);
    active_non_sink_arc_count = makespan_to_non_sink_arc_count.at(makespan);
}

bool HybridFlowPlanner::block_arc(NodeIndex tail,
                                     NodeIndex head,
                                     ArcIndex *blocked_arc) {
    ArcIndex arc = -1;
    if (tail >= 0 && tail < static_cast<NodeIndex>(out_arcs.size())) {
        for (ArcIndex candidate : out_arcs[tail]) {
            if (arc_head[candidate] == head) {
                arc = candidate;
                break;
            }
        }
    }
    if (arc < 0 || arc >= active_non_sink_arc_count) {
        return false;
    }
    arc_cap.at(arc) = 0;
    deleted_arcs.push_back(arc);
    if (blocked_arc != nullptr) {
        *blocked_arc = arc;
    }
    return true;
}

void HybridFlowPlanner::block_vertex_reservation(int assigned_id,
                                                     const Location &location,
                                                     int time) {
    if (time < 0 || time > current_makespan) {
        return;
    }
    const NodeIndex in = find_node(EarliestFailMinRCDirectFlowNode::IN, time, location);
    const NodeIndex out = find_node(EarliestFailMinRCDirectFlowNode::OUT, time, location);
    if (in < 0 || out < 0 || in >= active_node_count || out >= active_node_count) {
        return;
    }
    ArcIndex blocked_arc;
    if (block_arc(in, out, &blocked_arc)) {
        blocked_assigned_reservations.push_back(
                {RealizationConflict(assigned_id, location, time), blocked_arc});
    }
}

void HybridFlowPlanner::block_edge_reservation(int assigned_id,
                                                   const Location &from,
                                                   const Location &to,
                                                   int time) {
    if (time < 0 || time >= current_makespan || from == to) {
        return;
    }

    // If the assigned agent uses from -> to, an unassigned agent may not use
    // the opposite transition to -> from during the same timestep.
    const NodeIndex reverse_tail = find_node(EarliestFailMinRCDirectFlowNode::OUT, time, to);
    const NodeIndex reverse_head = find_node(EarliestFailMinRCDirectFlowNode::IN, time + 1, from);
    if (reverse_tail < 0 || reverse_head < 0 ||
        reverse_tail >= active_node_count || reverse_head >= active_node_count) {
        return;
    }

    ArcIndex blocked_arc;
    if (block_arc(reverse_tail, reverse_head, &blocked_arc)) {
        blocked_assigned_reservations.push_back(
                {RealizationConflict(assigned_id, from, to, time), blocked_arc});
    }
}

void HybridFlowPlanner::apply_assigned_reservations(
        const vector<shared_ptr<TimedPath>> &paths_to_realize) {
    blocked_assigned_reservations.clear();
    minimum_conflict_cut_realization_constraints.clear();

    if (verbose) {
        std::cout << "Applying assigned-agent reservations to the direct flow network:"
                  << std::endl;
    }

    for (int assigned_id = 0; assigned_id < static_cast<int>(paths_to_realize.size()); ++assigned_id) {
        const auto &path = paths_to_realize.at(assigned_id);
        if (verbose) {
            std::cout << "  a" << assigned_id << ": " << *path << std::endl;
        }
        const int last_move_time = std::min(current_makespan, path->back().time);
        for (int time = path->front().time; time <= current_makespan; ++time) {
            Location location = path->get_location_at_time(time);
            block_vertex_reservation(assigned_id, location, time);
            if (time < last_move_time) {
                block_edge_reservation(
                        assigned_id, location, path->get_location_at_time(time + 1), time);
            }
        }
    }

    if (verbose) {
        print_assigned_reservations();
        print_network_summary("after assigned reservations");
    }
}

HybridFlowPlanner::FlowQuantity
HybridFlowPlanner::required_flow() const {
    return num_of_agents + static_cast<FlowQuantity>(commit_edges_d_to_s.size());
}

HybridFlowPlanner::Result HybridFlowPlanner::find_paths(
        const vector<shared_ptr<TimedPath>> &paths_to_realize,
        const vector<shared_ptr<Constraints>> &constraints_table,
        int makespan) {
    FLOW_PROFILE_SCOPE("find_paths");
    Timer total_timer;
    total_timer.reset();
    if (stats) {
        stats->flow_checks++;
    }
    // The hybrid never passes committed edges (NRF-SAT clauses constrain the assigned agents instead).
    for (const auto &constraints : constraints_table) {
        if (constraints && !constraints->get_positive_edge_constraints().empty()) {
            throw std::logic_error("The optimized flow planner does not support committed edge constraints.");
        }
    }
    if (verbose) {
        std::cout << "Earliest-failure direct-flow planner request:" << std::endl;
        std::cout << "  full horizon: " << makespan << std::endl;
        std::cout << "  unassigned agents to route: " << num_of_agents << std::endl;
        std::cout << "  assigned paths to avoid: " << paths_to_realize.size() << std::endl;
    }

    // 1. Graph of this horizon (built only when the horizon grows).
    prepare_horizon(makespan);

    // 2. Reservations of this candidate, applied once for the full horizon. Every probe and the Min-RC
    //    stage see exactly the reserved arcs the original planner blocks for their prefix: an arc of a
    //    layer after the prefix cannot carry flow to the open sink layer.
    ++check_counter;
    {
        FLOW_PROFILE_SEGMENT(config_profile, "reservation_restore");
        for (ArcIndex arc : deleted_arcs) {
            arc_cap.at(arc) = CAP;
        }
        deleted_arcs.clear();
        FLOW_PROFILE_NEXT(config_profile, "reservation_apply");
        set_active_prefix(makespan);
        apply_assigned_reservations(paths_to_realize);
    }

    // 3. Full-horizon feasibility (solver 0).
    Result result = solve_prefix(0, makespan);

    if (result == INFEASIBLE) {
        FLOW_PROFILE_COUNT("failed_check", 1);
        int cut_solver = 0;
        int cut_horizon = makespan;
        if (use_earliest_failing_prefix) {
            FLOW_PROFILE_SEGMENT(search_profile, "binary_search");
            if (verbose) {
                std::cout << "Full horizon is infeasible; searching for the earliest failing prefix."
                          << std::endl;
            }

            // `best` holds the most recent failing prefix. Probes use the other solver so that the
            // maximum flow at t* can be reused by cut selection.
            int low = 0;
            int high = makespan;
            int best = 0;
            int prefix_probes = 0;
            // Warm start: t* is often the same as in the previous check. Failure is monotone in the prefix
            // (fails at t => fails at every t' > t), so probing p = previous t* (and p - 1 if p fails) only
            // narrows [low, high]; the binary search below then finds exactly the same earliest failing t*.
            if (warm_start_tstar && previous_tstar >= 0 && previous_tstar < makespan) {
                const int p = previous_tstar;
                const int solver = reuse_earliest_failing_probe ? 1 - best : 0;
                const Result at_p = solve_prefix(solver, p);
                prefix_probes++;
                if (at_p == SUCCESS) {
                    low = p + 1;
                } else {
                    high = p;
                    best = solver;
                    if (p > low) {
                        const int before = reuse_earliest_failing_probe ? 1 - best : 0;
                        const Result at_before = solve_prefix(before, p - 1);
                        prefix_probes++;
                        if (at_before == SUCCESS) {
                            low = p;
                        } else {
                            high = p - 1;
                            best = before;
                        }
                    }
                }
            }
            while (low < high) {
                const int middle = low + (high - low) / 2;
                FLOW_PROFILE_CONTEXT("probe", middle);
                FLOW_PROFILE_SCOPE("probe");
                FLOW_PROFILE_COUNT("binary_probe", 1);
                const int solver = reuse_earliest_failing_probe ? 1 - best : 0;
                const Result prefix_result = solve_prefix(solver, middle);
                prefix_probes++;
                if (prefix_result == SUCCESS) {
                    low = middle + 1;
                } else {
                    assert(prefix_result == INFEASIBLE);
                    high = middle;
                    best = solver;
                }
            }

            cut_horizon = low;
            previous_tstar = cut_horizon;
            FLOW_PROFILE_END(search_profile);
            cut_solver = best;
            if (reuse_earliest_failing_probe) {
#if STRIP_OWN_WITH_ORTOOLS
                if (!use_incremental_flow &&
                    (solver_open_layer[best] != cut_horizon || solver_check[best] != check_counter)) {
                    throw std::logic_error("The kept flow does not belong to the earliest failing prefix.");
                }
#endif
                set_active_prefix(cut_horizon);
                if (use_incremental_flow) {
                    maximum_flow = static_cast<FlowQuantity>(incremental_states.at(cut_horizon).value);
                }
#if STRIP_OWN_WITH_ORTOOLS
                else {
                    maximum_flow = static_cast<FlowQuantity>(ordinary_flow[best]->GetOptimalFlow());
                }
#endif
                if (maximum_flow == required_flow()) {
                    throw std::runtime_error("The earliest failing prefix unexpectedly became feasible.");
                }
            } else {
                cut_solver = 0;
                if (solve_prefix(cut_solver, cut_horizon) != INFEASIBLE) {
                    throw std::runtime_error("The earliest failing prefix unexpectedly became feasible.");
                }
            }
            if (verbose) {
                std::cout << "Earliest failing prefix t*: " << cut_horizon
                          << " (" << prefix_probes << " probes)" << std::endl;
            }
        } else {
            // Solver 0 still contains the failed full-horizon maximum flow from step 3.
            set_active_prefix(makespan);
            if (use_incremental_flow) {
                maximum_flow = static_cast<FlowQuantity>(incremental_states.at(makespan).value);
            }
#if STRIP_OWN_WITH_ORTOOLS
            else {
                maximum_flow = static_cast<FlowQuantity>(ordinary_flow[0]->GetOptimalFlow());
            }
#endif
            if (verbose) {
                std::cout << "Earliest-prefix probes disabled; selecting a cut at full horizon T="
                          << makespan << std::endl;
            }
        }

        FLOW_PROFILE_CONTEXT(use_minimum_conflict_cut ? "minrc" : "plain_cut", cut_horizon);
        if (use_minimum_conflict_cut) {
            FLOW_PROFILE_COUNT("minrc_call", 1);
            select_minimum_conflict_cut(cut_solver);
        } else {
            FLOW_PROFILE_COUNT("plain_cut_call", 1);
            select_ordinary_minimum_cut(cut_solver);
        }
        result = INFEASIBLE;
    }

    if (stats) {
        if (result == SUCCESS) {
            stats->flow_successes++;
        } else {
            stats->flow_failures++;
            if (result == INFEASIBLE) {
                stats->flow_infeasible++;
            } else {
                stats->flow_conflicts++;
            }
        }
    }
    if (result != SUCCESS) {
        if (stats) {
            stats->flow_total_seconds += total_timer.stop();
        }
        return result;
    }

    FLOW_PROFILE_COUNT("successful_check", 1);
    compute_selected_flow(0);
    FLOW_PROFILE_SEGMENT(extract_profile, "path_extraction");
    Timer stage_timer;
    stage_timer.reset();
    extract_agent_paths();
    if (stats) {
        stats->flow_extract_paths_seconds += stage_timer.stop();
        stats->flow_total_seconds += total_timer.stop();
    }
    if (verbose) {
        std::cout << "Extracted direct-flow unassigned-agent paths:" << std::endl;
        for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
            std::cout << "  u" << agent_id << ": " << *agent_paths.at(agent_id) << std::endl;
        }
    }
    return SUCCESS;
}

HybridFlowPlanner::Result HybridFlowPlanner::solve_prefix(int solver, int prefix) {
    if (use_incremental_flow) {
        return solve_prefix_incremental(solver, prefix);
    }
#if STRIP_OWN_WITH_ORTOOLS
    return solve_prefix_ortools(solver, prefix);
#else
    throw std::logic_error("OR-Tools max-flow fallback is unavailable in this build.");
#endif
}

bool HybridFlowPlanner::is_prefix_terminal(NodeIndex node, int prefix) const {
    const EarliestFailMinRCDirectFlowNode &info = idx_to_node[node];
    return info.type == EarliestFailMinRCDirectFlowNode::OUT && info.time == prefix;
}

const vector<HybridFlowPlanner::NodeIndex> &HybridFlowPlanner::incremental_source_side(int prefix) const {
    const IncrementalState &state = incremental_states.at(prefix);
    if (state.check != check_counter || state.value >= required_flow()) {
        throw std::logic_error("No failed incremental flow for this prefix in this check.");
    }
    return state.source_side;
}

void HybridFlowPlanner::incremental_cancel(IncrementalState &state, ArcIndex arc, int prefix) {
    // Every node carries at most one unit and the network is layered in time, so the unit through `arc`
    // is a single path source -> ... -> OUT(prefix, v); remove it.
    const ArcIndex arcs = static_cast<ArcIndex>(state.flow.size());
    NodeIndex node = arc_tail[arc];
    while (node != source_idx) {
        ArcIndex previous = -1;
        for (ArcIndex i = in_begin[node]; i < in_begin[node + 1]; ++i) {
            const ArcIndex candidate = in_list[i];
            if (candidate < arcs && state.flow[candidate]) { previous = candidate; break; }
        }
        if (previous < 0) throw std::logic_error("Incremental flow path is broken (backward).");
        state.flow[previous] = 0;
        node = arc_tail[previous];
    }
    node = arc_head[arc];
    while (!is_prefix_terminal(node, prefix)) {
        ArcIndex next = -1;
        for (ArcIndex candidate : out_arcs[node]) {
            if (candidate < arcs && state.flow[candidate]) { next = candidate; break; }
        }
        if (next < 0) throw std::logic_error("Incremental flow path is broken (forward).");
        state.flow[next] = 0;
        node = arc_head[next];
    }
    state.flow[arc] = 0;
    --state.value;
    ++debug_counters.cancels;
}

bool HybridFlowPlanner::incremental_augment_dfs(IncrementalState &state, int prefix) {
    // This version: one depth-first search in the residual network of the prefix (explicit stack, no
    // recursion). It follows one route to the last layer first (the wait arc is tried first), so on a big,
    // mostly free network it finds a path after about 2 * prefix steps instead of visiting everything.
    // On failure it has visited exactly the nodes reachable from the source, i.e. the same source side
    // as the BFS. Any augmenting path gives a maximum flow with the same value and source side.
    const ArcIndex arcs = static_cast<ArcIndex>(state.flow.size());
    const std::size_t nodes = idx_to_node.size();
    if (inc_mark.size() < nodes) {
        inc_mark.assign(nodes, 0);
        inc_parent.assign(nodes, -1);
        inc_parent_forward.assign(nodes, 0);
        inc_stamp = 0;
    }
    if (dfs_cursor.size() < nodes) {
        dfs_cursor.assign(nodes, 0);
    }
    if (++inc_stamp == std::numeric_limits<int>::max()) {
        std::fill(inc_mark.begin(), inc_mark.end(), 0);
        inc_stamp = 1;
    }
    inc_queue.clear();                  // every visited node (the source side on failure)
    dfs_stack.clear();
    inc_queue.push_back(source_idx);
    inc_mark[source_idx] = inc_stamp;
    dfs_cursor[source_idx] = 0;
    dfs_stack.push_back(source_idx);
    NodeIndex found = -1;
    while (!dfs_stack.empty()) {
        const NodeIndex node = dfs_stack.back();
        const vector<ArcIndex> &outs = out_arcs[node];
        const std::size_t out_count = outs.size();
        std::size_t &cursor = dfs_cursor[node];
        NodeIndex next = -1;
        while (true) {
            if (cursor < out_count) {
                const ArcIndex arc = outs[cursor++];
                if (arc >= arcs || state.flow[arc] || arc_cap[arc] <= 0) continue;
                const NodeIndex head = arc_head[arc];
                if (inc_mark[head] == inc_stamp) continue;
                inc_mark[head] = inc_stamp;
                inc_parent[head] = arc;
                inc_parent_forward[head] = 1;
                next = head;
                break;
            }
            const ArcIndex i = in_begin[node] + static_cast<ArcIndex>(cursor - out_count);
            if (i >= in_begin[node + 1]) break;
            ++cursor;
            const ArcIndex arc = in_list[i];
            if (arc >= arcs || !state.flow[arc]) continue;
            const NodeIndex tail = arc_tail[arc];
            if (inc_mark[tail] == inc_stamp) continue;
            inc_mark[tail] = inc_stamp;
            inc_parent[tail] = arc;
            inc_parent_forward[tail] = 0;
            next = tail;
            break;
        }
        if (next < 0) {
            dfs_stack.pop_back();
            continue;
        }
        inc_queue.push_back(next);
        if (inc_parent_forward[next] && is_prefix_terminal(next, prefix)) {
            found = next;
            break;
        }
        dfs_cursor[next] = 0;
        dfs_stack.push_back(next);
    }
    debug_counters.bfs_nodes += static_cast<long long>(inc_queue.size());
    if (found < 0) {
        state.source_side = inc_queue;
        ++debug_counters.failed_bfs;
        return false;
    }
    ++debug_counters.augments;
    NodeIndex node = found;
    while (node != source_idx) {
        const ArcIndex arc = inc_parent[node];
        if (inc_parent_forward[node]) {
            state.flow[arc] = 1;
            node = arc_tail[arc];
        } else {
            state.flow[arc] = 0;
            node = arc_head[arc];
        }
    }
    ++state.value;
    return true;
}

bool HybridFlowPlanner::incremental_augment(IncrementalState &state, int prefix) {
    if (use_dfs_augment) {
        return incremental_augment_dfs(state, prefix);
    }
    // One BFS in the residual network of the prefix. On failure, the visited nodes are the source side.
    const ArcIndex arcs = static_cast<ArcIndex>(state.flow.size());
    const std::size_t nodes = idx_to_node.size();
    if (inc_mark.size() < nodes) {
        inc_mark.assign(nodes, 0);
        inc_parent.assign(nodes, -1);
        inc_parent_forward.assign(nodes, 0);
        inc_stamp = 0;
    }
    if (++inc_stamp == std::numeric_limits<int>::max()) {
        std::fill(inc_mark.begin(), inc_mark.end(), 0);
        inc_stamp = 1;
    }
    inc_queue.clear();
    inc_queue.push_back(source_idx);
    inc_mark[source_idx] = inc_stamp;
    NodeIndex found = -1;
    for (std::size_t head = 0; head < inc_queue.size() && found < 0; ++head) {
        const NodeIndex node = inc_queue[head];
        for (ArcIndex arc : out_arcs[node]) {
            if (arc >= arcs || state.flow[arc] || arc_cap[arc] <= 0) continue;
            const NodeIndex next = arc_head[arc];
            if (inc_mark[next] == inc_stamp) continue;
            inc_mark[next] = inc_stamp;
            inc_parent[next] = arc;
            inc_parent_forward[next] = 1;
            inc_queue.push_back(next);
            if (is_prefix_terminal(next, prefix)) { found = next; break; }
        }
        if (found >= 0) break;
        for (ArcIndex i = in_begin[node]; i < in_begin[node + 1]; ++i) {
            const ArcIndex arc = in_list[i];
            if (arc >= arcs || !state.flow[arc]) continue;
            const NodeIndex previous = arc_tail[arc];
            if (inc_mark[previous] == inc_stamp) continue;
            inc_mark[previous] = inc_stamp;
            inc_parent[previous] = arc;
            inc_parent_forward[previous] = 0;
            inc_queue.push_back(previous);
        }
    }
    debug_counters.bfs_nodes += static_cast<long long>(inc_queue.size());
    if (found < 0) {
        state.source_side = inc_queue;   // everything reachable from the source in the residual network
        ++debug_counters.failed_bfs;
        return false;
    }
    ++debug_counters.augments;
    NodeIndex node = found;
    while (node != source_idx) {
        const ArcIndex arc = inc_parent[node];
        if (inc_parent_forward[node]) {
            state.flow[arc] = 1;
            node = arc_tail[arc];
        } else {
            state.flow[arc] = 0;
            node = arc_head[arc];
        }
    }
    ++state.value;
    return true;
}

void HybridFlowPlanner::ensure_in_arcs() {
    if (in_arcs_built_for == arc_tail.size() && in_begin.size() == out_arcs.size() + 1) {
        return;
    }
    in_begin.assign(out_arcs.size() + 1, 0);
    for (ArcIndex arc = 0; arc < static_cast<ArcIndex>(arc_tail.size()); ++arc) {
        ++in_begin[arc_head[arc] + 1];
    }
    for (std::size_t n = 0; n < out_arcs.size(); ++n) in_begin[n + 1] += in_begin[n];
    in_list.assign(arc_tail.size(), -1);
    vector<ArcIndex> fill(in_begin.begin(), in_begin.end() - 1);
    for (ArcIndex arc = 0; arc < static_cast<ArcIndex>(arc_tail.size()); ++arc) {
        in_list[fill[arc_head[arc]]++] = arc;
    }
    in_arcs_built_for = arc_tail.size();
}

HybridFlowPlanner::LexicographicCapacity HybridFlowPlanner::own_minrc_dfs(
        NodeIndex node, LexicographicCapacity limit, int prefix, ArcIndex arcs,
        const vector<LexicographicCapacity> &penalty, LexicographicCapacity multiplier) {
    if (is_prefix_terminal(node, prefix)) {
        return limit;   // OUT(t*, v) is entered only through its vertex arc (capacity <= M): never binding
    }
    const std::size_t out_degree = out_arcs[node].size();
    const std::size_t in_degree = static_cast<std::size_t>(in_begin[node + 1] - in_begin[node]);
    for (std::size_t &i = own_ptr[node]; i < out_degree + in_degree; ++i) {
        bool forward = i < out_degree;
        ArcIndex arc = forward ? out_arcs[node][i] : in_list[in_begin[node] + static_cast<ArcIndex>(i - out_degree)];
        if (arc >= arcs) continue;
        LexicographicCapacity residual;
        NodeIndex next;
        if (forward) {
            const LexicographicCapacity capacity = arc_cap[arc] > 0
                    ? static_cast<LexicographicCapacity>(arc_cap[arc]) * multiplier : penalty[arc];
            residual = capacity - own_w[arc];
            next = arc_head[arc];
        } else {
            residual = own_w[arc];
            next = arc_tail[arc];
        }
        if (residual <= 0 || own_level_stamp[next] != own_phase || own_level[next] != own_level[node] + 1) continue;
        const LexicographicCapacity pushed =
                own_minrc_dfs(next, std::min(limit, residual), prefix, arcs, penalty, multiplier);
        if (pushed > 0) {
            if (forward) own_w[arc] += pushed; else own_w[arc] -= pushed;
            return pushed;
        }
    }
    return 0;
}

HybridFlowPlanner::LexicographicCapacity HybridFlowPlanner::own_minrc_flow(
        const vector<LexicographicCapacity> &penalty, LexicographicCapacity multiplier,
        vector<NodeIndex> &source_side) {
    // Weighted network of prefix t*: open arc cap * M, blocked arc = its reservation count. The kept unit
    // flow times M is feasible (value maximum_flow * M); Dinic phases add the rest (about the number of
    // realization conflicts). The final residual-reachable set is the source side of the Min-RC cut.
    const int prefix = current_makespan;
    const IncrementalState &state = incremental_states.at(prefix);
    const ArcIndex arcs = makespan_to_non_sink_arc_count.at(prefix);
    ensure_in_arcs();
    own_w.assign(arcs, 0);
    for (ArcIndex arc = 0; arc < arcs; ++arc) {
        if (state.flow[arc]) own_w[arc] = multiplier;
    }
    const std::size_t nodes = idx_to_node.size();
    if (own_level.size() < nodes) {
        own_level.assign(nodes, 0);
        own_level_stamp.assign(nodes, 0);
        own_ptr.assign(nodes, 0);
        own_phase = 0;
    }
    LexicographicCapacity pushed_total = 0;
    const LexicographicCapacity infinite = std::numeric_limits<LexicographicCapacity>::max() / 4;
    if (const char *debug = std::getenv("UA_FLOW_COUNTS"); debug && std::string(debug) != "0") {
        // Area A: reachable from the source in the unit residual network (the failed check's source side).
        // Area B: nodes that can still reach a free end point: reverse search in the same residual network.
        const NodeIndex limit = makespan_to_node_count.at(prefix);
        ++debug_counters.own_calls;
        debug_counters.prefix_nodes += limit;
        long long a_count = 0;
        for (NodeIndex n : state.source_side) if (n < limit) ++a_count;
        debug_counters.area_a += a_count;
        vector<char> in_b(limit, 0);
        vector<NodeIndex> stack;
        for (NodeIndex n = 0; n < limit; ++n) {
            if (is_prefix_terminal(n, prefix)) {
                bool used = false;
                for (ArcIndex i = in_begin[n]; i < in_begin[n + 1]; ++i) if (in_list[i] < arcs && state.flow[in_list[i]]) used = true;
                if (!used) { in_b[n] = 1; stack.push_back(n); }
            }
        }
        while (!stack.empty()) {
            const NodeIndex y = stack.back(); stack.pop_back();
            for (ArcIndex i = in_begin[y]; i < in_begin[y + 1]; ++i) {
                const ArcIndex arc = in_list[i];
                const NodeIndex x = arc_tail[arc];
                if (arc < arcs && !state.flow[arc] && arc_cap[arc] > 0 && x < limit && !in_b[x]) { in_b[x] = 1; stack.push_back(x); }
            }
            for (ArcIndex arc : out_arcs[y]) {
                const NodeIndex x = arc_head[arc];
                if (arc < arcs && state.flow[arc] && x < limit && !in_b[x]) { in_b[x] = 1; stack.push_back(x); }
            }
        }
        long long b_count = 0;
        for (NodeIndex n = 0; n < limit; ++n) b_count += in_b[n];
        debug_counters.area_b += b_count;
    }
    vector<NodeIndex> queue;
    while (true) {
        ++own_phase;
        queue.clear();
        queue.push_back(source_idx);
        own_level_stamp[source_idx] = own_phase;
        own_level[source_idx] = 0;
        bool reached = false;
        for (std::size_t head = 0; head < queue.size(); ++head) {
            const NodeIndex node = queue[head];
            if (is_prefix_terminal(node, prefix)) { reached = true; continue; }
            for (ArcIndex arc : out_arcs[node]) {
                if (arc >= arcs) continue;
                const LexicographicCapacity capacity = arc_cap[arc] > 0
                        ? static_cast<LexicographicCapacity>(arc_cap[arc]) * multiplier : penalty[arc];
                const NodeIndex next = arc_head[arc];
                if (capacity - own_w[arc] <= 0 || own_level_stamp[next] == own_phase) continue;
                own_level_stamp[next] = own_phase;
                own_level[next] = own_level[node] + 1;
                queue.push_back(next);
            }
            for (ArcIndex i = in_begin[node]; i < in_begin[node + 1]; ++i) {
                const ArcIndex arc = in_list[i];
                if (arc >= arcs || own_w[arc] <= 0) continue;
                const NodeIndex previous = arc_tail[arc];
                if (own_level_stamp[previous] == own_phase) continue;
                own_level_stamp[previous] = own_phase;
                own_level[previous] = own_level[node] + 1;
                queue.push_back(previous);
            }
        }
        ++debug_counters.phases;
        debug_counters.phase_nodes += static_cast<long long>(queue.size());
        if (!reached) {
            source_side = queue;
            break;
        }
        for (NodeIndex node : queue) own_ptr[node] = 0;
        while (true) {
            const LexicographicCapacity pushed = own_minrc_dfs(source_idx, infinite, prefix, arcs, penalty, multiplier);
            if (pushed <= 0) break;
            pushed_total += pushed;
        }
    }
    return static_cast<LexicographicCapacity>(maximum_flow) * multiplier + pushed_total;
}

HybridFlowPlanner::LexicographicCapacity HybridFlowPlanner::strip_dfs(
        NodeIndex node, LexicographicCapacity limit, ArcIndex arcs, const vector<LexicographicCapacity> &penalty,
        LexicographicCapacity multiplier, const IncrementalState &state) {
    const std::size_t out_degree = out_arcs[node].size();
    const std::size_t in_degree = static_cast<std::size_t>(in_begin[node + 1] - in_begin[node]);
    for (std::size_t &i = strip_ptr[node]; i < out_degree + in_degree; ++i) {
        const bool forward = i < out_degree;
        const ArcIndex arc = forward ? out_arcs[node][i] : in_list[in_begin[node] + static_cast<ArcIndex>(i - out_degree)];
        if (arc >= arcs) continue;
        const LexicographicCapacity capacity = arc_cap[arc] > 0
                ? static_cast<LexicographicCapacity>(arc_cap[arc]) * multiplier : penalty[arc];
        const LexicographicCapacity used = (state.flow[arc] ? multiplier : 0) + strip_delta[arc];
        const LexicographicCapacity residual = forward ? capacity - used : used;
        const NodeIndex next = forward ? arc_head[arc] : arc_tail[arc];
        if (residual <= 0 || strip_region[next] == 1) continue;
        LexicographicCapacity pushed = 0;
        if (strip_region[next] == 2) {
            if (strip_level[node] + 1 != strip_sink_level) continue;
            pushed = std::min(limit, residual);
        } else {
            if (strip_stamp[next] != strip_phase || strip_level[next] != strip_level[node] + 1) continue;
            pushed = strip_dfs(next, std::min(limit, residual), arcs, penalty, multiplier, state);
        }
        if (pushed > 0) {
            strip_delta[arc] += forward ? pushed : -pushed;
            strip_touched.push_back(arc);
            return pushed;
        }
    }
    return 0;
}

HybridFlowPlanner::LexicographicCapacity HybridFlowPlanner::strip_minrc_flow(
        const vector<LexicographicCapacity> &penalty, LexicographicCapacity multiplier,
        vector<NodeIndex> &source_side) {
    // Weighted network of prefix t*: open arc cap * M, blocked arc = its reservation count. After M * (unit max
    // flow), the open arcs inside area A and area B can never be cut, so A acts as one source and B as one sink:
    // only blocked arcs leave A, and the Min-RC wall lies in the strip between A and B.
    const int prefix = current_makespan;
    const IncrementalState &state = incremental_states.at(prefix);
    const ArcIndex arcs = makespan_to_non_sink_arc_count.at(prefix);
    const NodeIndex limit = makespan_to_node_count.at(prefix);
    ensure_in_arcs();
    strip_region.assign(limit, 0);
    for (NodeIndex node : state.source_side) {
        if (node >= 0 && node < limit) strip_region[node] = 1;
    }
    // Area B: reverse search from the free end points over residual moves of the unit flow.
    vector<NodeIndex> stack;
    for (NodeIndex node = 0; node < limit; ++node) {
        if (!is_prefix_terminal(node, prefix)) continue;
        bool used = false;
        for (ArcIndex i = in_begin[node]; i < in_begin[node + 1]; ++i) {
            if (in_list[i] < arcs && state.flow[in_list[i]]) { used = true; break; }
        }
        if (used) continue;
        if (strip_region[node] == 1) throw std::logic_error("Strip Min-RC: a free end point is in area A.");
        strip_region[node] = 2;
        stack.push_back(node);
    }
    while (!stack.empty()) {
        const NodeIndex node = stack.back();
        stack.pop_back();
        for (ArcIndex i = in_begin[node]; i < in_begin[node + 1]; ++i) {
            const ArcIndex arc = in_list[i];
            if (arc >= arcs || state.flow[arc] || arc_cap[arc] <= 0) continue;
            const NodeIndex previous = arc_tail[arc];
            if (strip_region[previous] == 2) continue;
            if (strip_region[previous] == 1) throw std::logic_error("Strip Min-RC: areas A and B overlap.");
            strip_region[previous] = 2;
            stack.push_back(previous);
        }
        for (ArcIndex arc : out_arcs[node]) {
            if (arc >= arcs || !state.flow[arc]) continue;
            const NodeIndex previous = arc_head[arc];
            if (strip_region[previous] == 2) continue;
            if (strip_region[previous] == 1) throw std::logic_error("Strip Min-RC: areas A and B overlap.");
            strip_region[previous] = 2;
            stack.push_back(previous);
        }
    }
    // Exits of the source: blocked arcs leaving area A.
    strip_exits.clear();
    for (ArcIndex arc : deleted_arcs) {
        if (arc < arcs && strip_region[arc_tail[arc]] == 1 && strip_region[arc_head[arc]] != 1 && penalty[arc] > 0) {
            strip_exits.push_back(arc);
        }
    }
    std::sort(strip_exits.begin(), strip_exits.end());
    strip_exits.erase(std::unique(strip_exits.begin(), strip_exits.end()), strip_exits.end());
    if (strip_delta.size() < static_cast<std::size_t>(arcs)) strip_delta.resize(arcs, 0);
    if (strip_level.size() < static_cast<std::size_t>(limit)) {
        strip_level.assign(limit, 0);
        strip_stamp.assign(limit, 0);
        strip_ptr.assign(limit, 0);
        strip_phase = 0;
    }
    strip_touched.clear();
    auto residual_forward = [&](ArcIndex arc) {
        const LexicographicCapacity capacity = arc_cap[arc] > 0
                ? static_cast<LexicographicCapacity>(arc_cap[arc]) * multiplier : penalty[arc];
        return capacity - ((state.flow[arc] ? multiplier : 0) + strip_delta[arc]);
    };
    LexicographicCapacity pushed_total = 0;
    while (true) {
        ++strip_phase;
        strip_queue.clear();
        strip_sink_level = -1;
        for (ArcIndex arc : strip_exits) {
            if (residual_forward(arc) <= 0) continue;
            const NodeIndex next = arc_head[arc];
            if (strip_region[next] == 2) { strip_sink_level = 1; continue; }
            if (strip_stamp[next] == strip_phase) continue;
            strip_stamp[next] = strip_phase;
            strip_level[next] = 1;
            strip_queue.push_back(next);
        }
        for (std::size_t head = 0; head < strip_queue.size(); ++head) {
            const NodeIndex node = strip_queue[head];
            if (strip_sink_level >= 0 && strip_level[node] >= strip_sink_level) continue;
            for (ArcIndex arc : out_arcs[node]) {
                if (arc >= arcs || residual_forward(arc) <= 0) continue;
                const NodeIndex next = arc_head[arc];
                if (strip_region[next] == 1) continue;
                if (strip_region[next] == 2) {
                    if (strip_sink_level < 0) strip_sink_level = strip_level[node] + 1;
                    continue;
                }
                if (strip_stamp[next] == strip_phase) continue;
                strip_stamp[next] = strip_phase;
                strip_level[next] = strip_level[node] + 1;
                strip_queue.push_back(next);
            }
            for (ArcIndex i = in_begin[node]; i < in_begin[node + 1]; ++i) {
                const ArcIndex arc = in_list[i];
                if (arc >= arcs || (state.flow[arc] ? multiplier : 0) + strip_delta[arc] <= 0) continue;
                const NodeIndex next = arc_tail[arc];
                if (strip_region[next] == 1) continue;
                if (strip_region[next] == 2) {
                    if (strip_sink_level < 0) strip_sink_level = strip_level[node] + 1;
                    continue;
                }
                if (strip_stamp[next] == strip_phase) continue;
                strip_stamp[next] = strip_phase;
                strip_level[next] = strip_level[node] + 1;
                strip_queue.push_back(next);
            }
        }
        ++debug_counters.phases;
        debug_counters.phase_nodes += static_cast<long long>(strip_queue.size());
        if (strip_sink_level < 0) {
            source_side.clear();
            for (NodeIndex node : state.source_side) {
                if (node >= 0 && node < limit) source_side.push_back(node);
            }
            source_side.insert(source_side.end(), strip_queue.begin(), strip_queue.end());
            break;
        }
        for (NodeIndex node : strip_queue) strip_ptr[node] = 0;
        std::size_t exit = 0;
        while (exit < strip_exits.size()) {
            const ArcIndex arc = strip_exits[exit];
            const LexicographicCapacity residual = residual_forward(arc);
            const NodeIndex next = arc_head[arc];
            LexicographicCapacity pushed = 0;
            if (residual > 0) {
                if (strip_region[next] == 2) {
                    if (strip_sink_level == 1) pushed = residual;
                } else if (strip_stamp[next] == strip_phase && strip_level[next] == 1) {
                    pushed = strip_dfs(next, residual, arcs, penalty, multiplier, state);
                }
            }
            if (pushed > 0) {
                strip_delta[arc] += pushed;
                strip_touched.push_back(arc);
                pushed_total += pushed;
            } else {
                ++exit;
            }
        }
    }
    for (ArcIndex arc : strip_touched) strip_delta[arc] = 0;
    ++debug_counters.own_calls;
    return static_cast<LexicographicCapacity>(maximum_flow) * multiplier + pushed_total;
}

HybridFlowPlanner::Result HybridFlowPlanner::solve_prefix_incremental(int solver, int prefix) {
    const ArcIndex arcs = makespan_to_non_sink_arc_count.at(prefix);
    IncrementalState &state = incremental_states[prefix];
    if (state.flow.size() != static_cast<std::size_t>(arcs)) {
        // Keep as many prefixes as fit in about 256 MB of flow arrays (at least 8, at most 256);
        // drop the least recently used one beyond that (same rule as the incremental version).
        const std::size_t max_states = std::min<std::size_t>(
                256, std::max<std::size_t>(8, (static_cast<std::size_t>(256) << 20) /
                                                  std::max<std::size_t>(1, static_cast<std::size_t>(arcs))));
        if (incremental_states.size() > max_states) {
            auto oldest = incremental_states.end();
            for (auto it = incremental_states.begin(); it != incremental_states.end(); ++it) {
                if (it->first != prefix && (oldest == incremental_states.end() || it->second.check < oldest->second.check)) {
                    oldest = it;
                }
            }
            if (oldest != incremental_states.end()) incremental_states.erase(oldest);
        }
        // First time for this prefix (this version): no OR-Tools. Start from the empty flow and add
        // BFS augmenting paths until the flow is maximum. The maximum-flow value and the residual source
        // side are the same for every maximum flow, so the search is identical to the OR-Tools start.
        const auto debug_start = DebugClock::now();
        ++debug_counters.fresh;
        Timer stage_timer;
        stage_timer.reset();
        ensure_in_arcs();
        state.flow.assign(arcs, 0);
        state.value = 0;
        state.check = check_counter;
        state.source_side.clear();
        while (state.value < required_flow()) {
            if (!incremental_augment(state, prefix)) break;
        }
        if (stats) {
            stats->flow_max_flow_seconds += stage_timer.stop();
        }
        debug_counters.fresh_s += std::chrono::duration<double>(DebugClock::now() - debug_start).count();
    } else if (state.check != check_counter) {
        const auto debug_start = DebugClock::now();
        ++debug_counters.repairs;
        Timer stage_timer;
        stage_timer.reset();
        ensure_in_arcs();
        for (ArcIndex arc : deleted_arcs) {
            if (arc < arcs && state.flow[arc]) {
                incremental_cancel(state, arc, prefix);
            }
        }
        state.check = check_counter;
        state.source_side.clear();
        while (state.value < required_flow()) {
            if (!incremental_augment(state, prefix)) break;
        }
        if (stats) {
            stats->flow_max_flow_seconds += stage_timer.stop();
        }
        debug_counters.repair_s += std::chrono::duration<double>(DebugClock::now() - debug_start).count();
    }
    set_active_prefix(prefix);
    maximum_flow = static_cast<FlowQuantity>(state.value);
    const Result result = state.value >= required_flow() ? SUCCESS : INFEASIBLE;
#if STRIP_OWN_WITH_ORTOOLS
    if (incremental_verify) {
        const Result reference = solve_prefix_ortools(solver, prefix);
        if (reference != result || maximum_flow != state.value) {
            throw std::logic_error("Incremental max-flow value differs from OR-Tools.");
        }
        if (result == INFEASIBLE) {
            vector<NodeIndex> reference_side;
            ordinary_flow[solver]->GetSourceSideMinCut(&reference_side);
            const NodeIndex limit = makespan_to_node_count.at(prefix);
            vector<NodeIndex> a, b;
            for (NodeIndex n : reference_side) if (n < limit) a.push_back(n);
            for (NodeIndex n : state.source_side) if (n < limit) b.push_back(n);
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            if (a != b) throw std::logic_error("Incremental source side differs from OR-Tools.");
        }
        set_active_prefix(prefix);
        maximum_flow = static_cast<FlowQuantity>(state.value);
    }
#endif
    return result;
}

#if STRIP_OWN_WITH_ORTOOLS
HybridFlowPlanner::Result HybridFlowPlanner::solve_prefix_ortools(int solver, int prefix) {
    // Brings the solver's capacities to (this check's reservations, layer `prefix` open) by changing only
    // the arcs that differ, then solves. Arcs of layers after `prefix` and the OUT -> L_t arcs of closed
    // layers stay open but lead nowhere: flow cannot reach the sink from them, so the maximum flow and the
    // reachable (source-side) part of layers 0..prefix are exactly those of the original prefix network.
    FLOW_PROFILE_SEGMENT(solve_profile, "maxflow_setup");
    FLOW_PROFILE_COUNT("solve_flow_call", 1);
    MaxFlow &flow = ordinary_solver(solver);
    if (solver_check[solver] != check_counter) {
        for (ArcIndex arc : solver_blocked[solver]) {
            flow.SetArcCapacity(permuted_arc(arc), CAP);
        }
        solver_blocked[solver] = deleted_arcs;
        for (ArcIndex arc : deleted_arcs) {
            flow.SetArcCapacity(permuted_arc(arc), 0);
        }
        solver_check[solver] = check_counter;
    }
    if (solver_open_layer[solver] != prefix) {
        if (solver_open_layer[solver] >= 0) {
            flow.SetArcCapacity(permuted_arc(layer_sink_arc(solver_open_layer[solver])), 0);
        }
        flow.SetArcCapacity(permuted_arc(layer_sink_arc(prefix)), num_of_agents);
        solver_open_layer[solver] = prefix;
    }

    FLOW_PROFILE_NEXT(solve_profile, "maxflow_solve");
    FLOW_PROFILE_COUNT("ordinary_maxflow", 1);
    ++debug_counters.ortools_solves;
    Timer stage_timer;
    stage_timer.reset();
    if (!flow.Solve()) {
        throw std::runtime_error("Direct-network max flow could not be computed.");
    }
    if (stats) {
        stats->flow_max_flow_seconds += stage_timer.stop();
    }
    FLOW_PROFILE_NEXT(solve_profile, "solve_other");
    set_active_prefix(prefix);
    maximum_flow = static_cast<FlowQuantity>(flow.GetOptimalFlow());
    if (verbose) {
        std::cout << "Direct-network max-flow feasibility check:" << std::endl;
        std::cout << "  required flow: " << required_flow() << std::endl;
        std::cout << "  achieved flow: " << maximum_flow << std::endl;
    }
    return maximum_flow == required_flow() ? SUCCESS : INFEASIBLE;
}
#endif

void HybridFlowPlanner::select_ordinary_minimum_cut(int solver) {
    FLOW_PROFILE_SCOPE("plain_cut");
    vector<NodeIndex> source_side_nodes;
    if (use_incremental_flow) {
        source_side_nodes = incremental_source_side(current_makespan);
    }
#if STRIP_OWN_WITH_ORTOOLS
    else {
        ordinary_flow[solver]->GetSourceSideMinCut(&source_side_nodes);
    }
#endif
    update_minimum_conflict_cut_realization_constraints(source_side_nodes, graph_node_count);
    if (verbose) {
        std::cout << "Ordinary source-side minimum cut:" << std::endl;
        std::cout << "  realization conflicts: "
                  << minimum_conflict_cut_realization_constraints.size() << std::endl;
        print_minimum_conflict_cut_realization_constraints();
    }
}

void HybridFlowPlanner::select_minimum_conflict_cut(int solver) {
    const auto debug_minrc_start = DebugClock::now();
    ++debug_counters.minrc;
    struct MinrcTimer { DebugClock::time_point s; ~MinrcTimer() { debug_counters.minrc_s += std::chrono::duration<double>(DebugClock::now() - s).count(); } } minrc_timer{debug_minrc_start};
    // Same lexicographic Min-RC cut as the original, on the network of prefix t* (= current_makespan):
    // capacity cap * M for every open arc of the prefix, the number of reservations for a blocked arc,
    // and 0 for everything outside the prefix.
    FLOW_PROFILE_SEGMENT(cut_profile, "minrc_setup");
    const NodeIndex augmented_num_nodes = graph_node_count;
    vector<NodeIndex> ordinary_source_side_nodes;
    if (use_incremental_flow) {
        ordinary_source_side_nodes = incremental_source_side(current_makespan);
    }
#if STRIP_OWN_WITH_ORTOOLS
    else {
        ordinary_flow[solver]->GetSourceSideMinCut(&ordinary_source_side_nodes);
    }
#endif
    update_minimum_conflict_cut_realization_constraints(
            ordinary_source_side_nodes, augmented_num_nodes);
    const size_t ordinary_source_side_conflict_count =
            minimum_conflict_cut_realization_constraints.size();

    const LexicographicCapacity capacity_multiplier =
            static_cast<LexicographicCapacity>(num_assigned_agents) *
            (2 * static_cast<LexicographicCapacity>(current_makespan) + 1) + 1;
    // The original network of prefix t* contains exactly the reservations whose arc lies in the prefix.
    size_t prefix_reservations = 0;
    for (const BlockedAssignedReservation &reservation : blocked_assigned_reservations) {
        if (reservation.arc >= 0 && reservation.arc < active_non_sink_arc_count) {
            prefix_reservations++;
        }
    }
    if (static_cast<LexicographicCapacity>(prefix_reservations) >= capacity_multiplier) {
        throw std::runtime_error(
                "The minimum-conflict cut multiplier does not dominate all reservation penalties.");
    }
    if (static_cast<LexicographicCapacity>(num_of_agents) >
        std::numeric_limits<LexicographicCapacity>::max() / capacity_multiplier) {
        throw std::overflow_error("The minimum-conflict cut capacity transformation overflowed.");
    }

    vector<LexicographicCapacity> reservation_penalty(active_non_sink_arc_count, 0);
    set<std::tuple<ArcIndex, int, int, int, int, int, int, int>> seen_reservations;
    for (const BlockedAssignedReservation &reservation : blocked_assigned_reservations) {
        const ArcIndex arc = reservation.arc;
        if (arc < 0 || arc >= active_non_sink_arc_count) {
            continue;
        }
        const RealizationConflict &conflict = reservation.conflict;
        auto key = std::make_tuple(
                arc,
                conflict.type == RealizationConflict::Type::Vertex ? 0 : 1,
                conflict.assigned_id,
                conflict.time,
                conflict.location1.x,
                conflict.location1.y,
                conflict.location2.x,
                conflict.location2.y);
        if (seen_reservations.insert(key).second) {
            reservation_penalty.at(arc)++;
        }
    }

    const bool own_minrc = use_incremental_flow && use_own_minrc;
    LexicographicCapacity own_encoded = 0;
    vector<NodeIndex> own_side;
    if (own_minrc) {
        Timer own_timer;
        own_timer.reset();
        const auto s0 = DebugClock::now();
        own_encoded = strip_minrc_flow(reservation_penalty, capacity_multiplier, own_side);
        debug_counters.solve_s += std::chrono::duration<double>(DebugClock::now() - s0).count();
        if (stats) {
            stats->flow_max_flow_seconds += own_timer.stop();
        }
    }
    LexicographicCapacity reference_encoded = 0;
    vector<NodeIndex> reference_side;
    if (!own_minrc || incremental_verify) {
#if STRIP_OWN_WITH_ORTOOLS
    MaxFlow &weighted = weighted_solver();
    for (ArcIndex arc = 0; arc < graph_non_sink_arc_count; ++arc) {
        LexicographicCapacity capacity = 0;
        if (arc < active_non_sink_arc_count) {
            capacity = arc_cap[arc] > 0
                       ? static_cast<LexicographicCapacity>(arc_cap[arc]) * capacity_multiplier
                       : reservation_penalty[arc];
        }
        weighted.SetArcCapacity(permuted_arc(arc), capacity);
    }
    // Only layer t* reaches the sink: its OUT -> L_t* arcs play the role of the original sink arcs
    // (cap * M each) and L_t* -> sink carries at most the total flow.
    const LexicographicCapacity total_capacity =
            static_cast<LexicographicCapacity>(num_of_agents) * capacity_multiplier;
    for (int layer = 0; layer <= graph_horizon; ++layer) {
        const bool open = layer == current_makespan;
        for (ArcIndex arc = sink_arc_begin[layer]; arc < sink_arc_begin[layer + 1]; ++arc) {
            weighted.SetArcCapacity(permuted_arc(arc),
                                    open ? static_cast<LexicographicCapacity>(CAP) * capacity_multiplier : 0);
        }
        weighted.SetArcCapacity(permuted_arc(layer_sink_arc(layer)), open ? total_capacity : 0);
    }
    weighted.SetArcCapacity(permuted_arc(total_source_arc), total_capacity);
    weighted.SetArcCapacity(permuted_arc(total_sink_arc), total_capacity);

    FLOW_PROFILE_NEXT(cut_profile, "minrc_weighted_solve");
    FLOW_PROFILE_COUNT("weighted_maxflow", 1);
    ++debug_counters.ortools_solves;
    Timer stage_timer;
    stage_timer.reset();
    if (!weighted.Solve()) {
        throw std::runtime_error("Minimum-realization-conflict cut flow could not be computed.");
    }
    if (stats) {
        stats->flow_max_flow_seconds += stage_timer.stop();
    }

    reference_encoded = weighted.GetOptimalFlow();
    weighted.GetSourceSideMinCut(&reference_side);
#else
        throw std::logic_error("OR-Tools Min-RC fallback is unavailable in this build.");
#endif
    }
    FLOW_PROFILE_NEXT(cut_profile, "minrc_extract_validate");
    if (own_minrc && incremental_verify) {
        const NodeIndex limit = makespan_to_node_count.at(current_makespan);
        vector<NodeIndex> a, b;
        for (NodeIndex n : reference_side) if (n < limit) a.push_back(n);
        for (NodeIndex n : own_side) if (n < limit) b.push_back(n);
        std::sort(a.begin(), a.end());
        std::sort(b.begin(), b.end());
        if (reference_encoded != own_encoded || a != b) {
            throw std::logic_error("Own Min-RC flow differs from OR-Tools.");
        }
    }
    const LexicographicCapacity encoded_cut_capacity = own_minrc ? own_encoded : reference_encoded;
    const LexicographicCapacity selected_original_cut_capacity =
            encoded_cut_capacity / capacity_multiplier;
    const LexicographicCapacity selected_realization_conflict_count =
            encoded_cut_capacity % capacity_multiplier;
    if (selected_original_cut_capacity != maximum_flow) {
        throw std::runtime_error(
                "Capacity transformation changed the primary minimum-cut objective.");
    }

    const vector<NodeIndex> &source_side_nodes = own_minrc ? own_side : reference_side;
    update_minimum_conflict_cut_realization_constraints(source_side_nodes, augmented_num_nodes);
    if (selected_realization_conflict_count !=
        static_cast<LexicographicCapacity>(minimum_conflict_cut_realization_constraints.size())) {
        throw std::runtime_error(
                "Decoded realization-conflict penalty does not match the selected cut.");
    }
    if (minimum_conflict_cut_realization_constraints.size() > ordinary_source_side_conflict_count) {
        throw std::runtime_error(
                "Minimum-conflict cut contains more conflicts than the ordinary source-side cut.");
    }

    if (verbose) {
        std::cout << "Minimum-realization-conflict cut selection:" << std::endl;
        std::cout << "  capacity multiplier M: " << capacity_multiplier << std::endl;
        std::cout << "  ordinary source-side cut conflict count: "
                  << ordinary_source_side_conflict_count << std::endl;
        std::cout << "  encoded cut capacity: " << encoded_cut_capacity << std::endl;
        std::cout << "  decoded original cut capacity: " << selected_original_cut_capacity << std::endl;
        std::cout << "  decoded realization-conflict count: "
                  << selected_realization_conflict_count << std::endl;
        std::cout << "  selected source-side cut nodes: " << source_side_nodes.size() << std::endl;
        print_minimum_conflict_cut_realization_constraints();
    }
}

void HybridFlowPlanner::compute_selected_flow(int solver) {
    // Flow of the successful full-horizon check (prefix = current_makespan), per non-sink arc.
    FLOW_PROFILE_SEGMENT(selected_profile, "selected_flow_copy");
    const ArcIndex prefix_arcs = active_non_sink_arc_count;
    arc_flow.assign(arc_tail.size(), 0);
    const FlowQuantity required_flow_value = required_flow();
    if (use_min_cost_flow) {
#if STRIP_OWN_WITH_ORTOOLS
        // The min-cost flow runs once per run, on the successful check. It gets exactly the network the
        // original planner builds for that check (prefix arcs, the sink arcs of layer T straight into the
        // sink, the two total arcs, in the same order), so it does the original's work and returns the
        // original's paths. On the shared graph it was much slower on large maps (many closed arcs).
        FLOW_PROFILE_NEXT(selected_profile, "mincost_setup");
        const NodeIndex node_count = active_node_count;
        const NodeIndex mc_source = node_count;
        const NodeIndex mc_sink = node_count + 1;
        const ArcIndex layer_begin = sink_arc_begin.at(current_makespan);
        const ArcIndex layer_end = sink_arc_begin.at(current_makespan + 1);
        const ArcIndex mc_num_arcs = prefix_arcs + (layer_end - layer_begin) + 2;
        Graph mc_graph(node_count + 2, mc_num_arcs);
        for (ArcIndex arc = 0; arc < prefix_arcs; ++arc) {
            mc_graph.AddArc(arc_tail[arc], arc_head[arc]);
        }
        for (ArcIndex arc = layer_begin; arc < layer_end; ++arc) {
            mc_graph.AddArc(sink_arc_tails[arc - graph_non_sink_arc_count], sink_idx);
        }
        mc_graph.AddArc(mc_source, source_idx);
        mc_graph.AddArc(sink_idx, mc_sink);
        vector<ArcIndex> mc_permutation;
        mc_graph.Build(&mc_permutation);
        auto mc_arc = [&mc_permutation](ArcIndex arc) {
            return arc < static_cast<ArcIndex>(mc_permutation.size()) ? mc_permutation[arc] : arc;
        };

        MinCostFlow min_cost_flow(&mc_graph);
        ArcIndex index = 0;
        for (ArcIndex arc = 0; arc < prefix_arcs; ++arc, ++index) {
            min_cost_flow.SetArcUnitCost(mc_arc(index), arc_cost[arc]);
            min_cost_flow.SetArcCapacity(mc_arc(index), arc_cap[arc]);
        }
        for (ArcIndex arc = layer_begin; arc < layer_end; ++arc, ++index) {
            min_cost_flow.SetArcUnitCost(mc_arc(index), NO_COST);
            min_cost_flow.SetArcCapacity(mc_arc(index), CAP);
        }
        for (int total = 0; total < 2; ++total, ++index) {
            min_cost_flow.SetArcUnitCost(mc_arc(index), NO_COST);
            min_cost_flow.SetArcCapacity(mc_arc(index), num_of_agents);
        }
        min_cost_flow.SetNodeSupply(mc_source, required_flow_value);
        min_cost_flow.SetNodeSupply(mc_sink, -required_flow_value);
        min_cost_flow.SetCheckFeasibility(false);

        Timer stage_timer;
        stage_timer.reset();
        FLOW_PROFILE_NEXT(selected_profile, "mincost_solve");
        FLOW_PROFILE_COUNT("mincost_solve", 1);
        if (!min_cost_flow.Solve()) {
            throw std::runtime_error("Direct-network min-cost flow could not be computed.");
        }
        if (stats) {
            stats->flow_max_flow_seconds += stage_timer.stop();
        }
        FLOW_PROFILE_NEXT(selected_profile, "selected_flow_copy");
        if (min_cost_flow.status() != operations_research::MinCostFlow::OPTIMAL) {
            throw std::runtime_error("Direct-network min-cost flow was not optimal.");
        }
        optimal_cost = static_cast<CostValue>(min_cost_flow.GetOptimalCost());
        for (ArcIndex arc = 0; arc < prefix_arcs; ++arc) {
            arc_flow[arc] = static_cast<FlowQuantity>(min_cost_flow.Flow(mc_arc(arc)));
        }
#else
        throw std::logic_error("Min-cost flow is unavailable in this OR-Tools-free build.");
#endif
    } else {
        if (use_incremental_flow) {
            const IncrementalState &state = incremental_states.at(current_makespan);
            for (ArcIndex arc = 0; arc < prefix_arcs; ++arc) {
                arc_flow[arc] = state.flow.at(arc);
            }
        }
#if STRIP_OWN_WITH_ORTOOLS
        else {
            const MaxFlow &flow = *ordinary_flow[solver];
            for (ArcIndex arc = 0; arc < prefix_arcs; ++arc) {
                arc_flow[arc] = static_cast<FlowQuantity>(flow.Flow(permuted_arc(arc)));
            }
        }
#endif
    }

    FLOW_PROFILE_NEXT(selected_profile, "swap_normalization");
    FLOW_PROFILE_COUNT("swap_normalization", 1);
    normalize_anonymous_swaps();
    if (verbose) {
        std::cout << (use_min_cost_flow ? "Direct min-cost flow succeeded" :
                                          "Direct maximum flow succeeded")
                  << ":" << std::endl;
        if (use_min_cost_flow) {
            std::cout << "  optimal unassigned movement cost: " << optimal_cost << std::endl;
        }
        print_selected_flow();
    }
}

void HybridFlowPlanner::update_minimum_conflict_cut_realization_constraints(
        const vector<NodeIndex> &source_side_nodes,
        NodeIndex augmented_node_count) {
    struct UpdateTimer { DebugClock::time_point s = DebugClock::now(); ~UpdateTimer() { debug_counters.update_s += std::chrono::duration<double>(DebugClock::now() - s).count(); } } update_timer;
    minimum_conflict_cut_realization_constraints.clear();

    vector<bool> on_source_side(augmented_node_count, false);
    for (NodeIndex node : source_side_nodes) {
        if (node >= 0 && node < augmented_node_count) {
            on_source_side.at(node) = true;
        }
    }

    set<std::tuple<int, int, int, int, int, int, int>> seen_conflicts;
    for (const BlockedAssignedReservation &reservation : blocked_assigned_reservations) {
        const ArcIndex arc = reservation.arc;
        if (arc < 0 || arc >= active_non_sink_arc_count) {
            continue;
        }

        const NodeIndex tail = arc_tail.at(arc);
        const NodeIndex head = arc_head.at(arc);
        if (!on_source_side.at(tail) || on_source_side.at(head)) {
            continue;
        }

        const RealizationConflict &conflict = reservation.conflict;
        auto key = std::make_tuple(
                conflict.type == RealizationConflict::Type::Vertex ? 0 : 1,
                conflict.assigned_id,
                conflict.time,
                conflict.location1.x,
                conflict.location1.y,
                conflict.location2.x,
                conflict.location2.y);
        if (seen_conflicts.insert(key).second) {
            minimum_conflict_cut_realization_constraints.push_back(conflict);
        }
    }

    std::sort(minimum_conflict_cut_realization_constraints.begin(),
              minimum_conflict_cut_realization_constraints.end(),
              [](const RealizationConflict &left, const RealizationConflict &right) {
                  auto key = [](const RealizationConflict &conflict) {
                      return std::make_tuple(
                              conflict.time,
                              conflict.assigned_id,
                              conflict.type == RealizationConflict::Type::Vertex ? 0 : 1,
                              conflict.location1.x,
                              conflict.location1.y,
                              conflict.location2.x,
                              conflict.location2.y);
                  };
                  return key(left) < key(right);
              });
}

#if STRIP_OWN_WITH_ORTOOLS
HybridFlowPlanner::ArcIndex HybridFlowPlanner::permuted_arc(ArcIndex arc) const {
    return arc < static_cast<int>(arc_permutation.size()) ? arc_permutation.at(arc) : arc;
}
#endif

void HybridFlowPlanner::normalize_anonymous_swaps() {
    // Opposite anonymous transitions x->y and y->x have exactly the same
    // occupancy effect as x->x and y->y. Replace them before assigning stable
    // labels to the flow units so the returned paths obey the MAPF swap rule.
    auto arc_between = [this](NodeIndex tail, NodeIndex head) -> ArcIndex {
        for (ArcIndex arc : out_arcs[tail]) {
            if (arc_head[arc] == head) {
                return arc;
            }
        }
        return -1;
    };
    for (int time = 0; time < current_makespan; ++time) {
        for (const auto &[first, second] : map_edges) {
            const NodeIndex first_out = find_node(EarliestFailMinRCDirectFlowNode::OUT, time, first);
            const NodeIndex second_out = find_node(EarliestFailMinRCDirectFlowNode::OUT, time, second);
            const NodeIndex first_in = find_node(EarliestFailMinRCDirectFlowNode::IN, time + 1, first);
            const NodeIndex second_in = find_node(EarliestFailMinRCDirectFlowNode::IN, time + 1, second);
            if (first_out < 0 || second_out < 0 || first_in < 0 || second_in < 0) {
                continue;
            }

            const ArcIndex first_to_second = arc_between(first_out, second_in);
            const ArcIndex second_to_first = arc_between(second_out, first_in);
            if (first_to_second < 0 || second_to_first < 0) {
                continue;
            }
            if (first_to_second >= static_cast<ArcIndex>(arc_flow.size()) ||
                second_to_first >= static_cast<ArcIndex>(arc_flow.size()) ||
                arc_flow.at(first_to_second) <= 0 || arc_flow.at(second_to_first) <= 0) {
                continue;
            }

            assert(has_arc(first_out, first_in));
            assert(has_arc(second_out, second_in));
            const ArcIndex first_wait = get_arc_idx(first_out, first_in);
            const ArcIndex second_wait = get_arc_idx(second_out, second_in);
            assert(arc_cap.at(first_wait) > 0 && arc_cap.at(second_wait) > 0);
            assert(arc_flow.at(first_wait) == 0 && arc_flow.at(second_wait) == 0);

            arc_flow.at(first_to_second)--;
            arc_flow.at(second_to_first)--;
            arc_flow.at(first_wait)++;
            arc_flow.at(second_wait)++;
            if (use_min_cost_flow) {
                optimal_cost -= 2 * UNIT_COST;
            }

            if (verbose) {
                std::cout << "  normalized anonymous swap at t=" << time << " over "
                          << first << "<->" << second << " into two waits" << std::endl;
            }
        }
    }
}

void HybridFlowPlanner::consume_flow_arc(NodeIndex tail, NodeIndex head) {
    ArcIndex arc = get_arc_idx(tail, head);
    if (arc < 0 || arc >= static_cast<ArcIndex>(arc_flow.size()) || arc_flow.at(arc) <= 0) {
        throw std::runtime_error("Selected direct-flow arc is unavailable during path extraction.");
    }
    arc_flow.at(arc)--;
}

HybridFlowPlanner::NodeIndex HybridFlowPlanner::consume_next_flow_arc(NodeIndex tail) {
    if (tail < 0 || tail >= static_cast<NodeIndex>(out_arcs.size()) || out_arcs[tail].empty()) {
        throw std::runtime_error("No outgoing direct-flow arcs exist during path extraction.");
    }

    bool found = false;
    NodeIndex selected_head = -1;
    ArcIndex selected_arc = -1;
    for (ArcIndex arc : out_arcs[tail]) {
        if (arc >= 0 && arc < static_cast<ArcIndex>(arc_flow.size()) && arc_flow.at(arc) > 0) {
            if (found) {
                throw std::runtime_error(
                        "Multiple selected direct-flow arcs leave one capacity-one vertex.");
            }
            found = true;
            selected_head = arc_head[arc];
            selected_arc = arc;
        }
    }
    if (!found) {
        throw std::runtime_error("No selected direct-flow arc remains during path extraction.");
    }
    arc_flow.at(selected_arc)--;
    return selected_head;
}

void HybridFlowPlanner::extract_agent_paths() {
    for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
        agent_paths.at(agent_id) = std::make_shared<TimedPath>();
        agent_paths.at(agent_id)->push_back(State(0, agent_start_locations.at(agent_id)));
        NodeIndex start_in = at_node(EarliestFailMinRCDirectFlowNode::IN, 0, agent_start_locations.at(agent_id));
        NodeIndex start_out = at_node(EarliestFailMinRCDirectFlowNode::OUT, 0, agent_start_locations.at(agent_id));
        consume_flow_arc(source_idx, start_in);
        consume_flow_arc(start_in, start_out);
    }

    realization_conflict = nullptr;
    for (int time = 1; time <= current_makespan; ++time) {
        for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
            const Location previous = agent_paths.at(agent_id)->back().location;
            NodeIndex previous_out = at_node(EarliestFailMinRCDirectFlowNode::OUT, time - 1, previous);
            auto committed_it = commit_edges_d_to_s.find(previous_out);
            NodeIndex next_in = committed_it == commit_edges_d_to_s.end()
                                ? consume_next_flow_arc(previous_out)
                                : committed_it->second;
            const EarliestFailMinRCDirectFlowNode &next_node = idx_to_node.at(next_in);
            if (next_node.type != EarliestFailMinRCDirectFlowNode::IN || next_node.time != time) {
                throw std::runtime_error("Direct-flow transition did not reach the next IN layer.");
            }
            agent_paths.at(agent_id)->push_back(State(time, next_node.location));
            NodeIndex next_out = consume_next_flow_arc(next_in);
            if (idx_to_node.at(next_out).type != EarliestFailMinRCDirectFlowNode::OUT) {
                throw std::runtime_error("Direct-flow vertex-capacity arc did not reach OUT.");
            }
        }
    }
}

const vector<RealizationConflict> &
HybridFlowPlanner::get_minimum_conflict_cut_realization_constraints() const {
    return minimum_conflict_cut_realization_constraints;
}

void HybridFlowPlanner::print_selected_flow() const {
    std::cout << "  positive direct-flow arcs:" << std::endl;
    bool found = false;
    for (ArcIndex arc = 0; arc < static_cast<ArcIndex>(arc_flow.size()); ++arc) {
        if (arc_flow.at(arc) <= 0) {
            continue;
        }
        found = true;
        std::cout << "    " << idx_to_node.at(arc_tail.at(arc)) << " -> "
                  << idx_to_node.at(arc_head.at(arc)) << " flow=" << arc_flow.at(arc)
                  << " cost=" << arc_cost.at(arc) << std::endl;
    }
    if (!found) {
        std::cout << "    none" << std::endl;
    }
}

void HybridFlowPlanner::print_network_summary(const string &stage) const {
    const FlowNetworkSnapshot snapshot = current_network_snapshot();
    std::cout << "Direct flow network summary (" << stage << "):" << std::endl;
    std::cout << "  time horizon: " << snapshot.time_horizon << std::endl;
    std::cout << "  nodes: " << snapshot.nodes << std::endl;
    std::cout << "  arcs: " << snapshot.arcs << std::endl;
    std::cout << "  map cells reached by network: " << snapshot.reached_locations << std::endl;
    std::cout << "  undirected map adjacencies: " << map_edges.size() << std::endl;
    std::cout << "  edge-gadget nodes: 0" << std::endl;
    std::cout << "  blocked arcs: " << snapshot.blocked_arcs << std::endl;
    std::cout << "  forced/commit edges: " << snapshot.forced_commit_edges << std::endl;
}

int HybridFlowPlanner::count_blocked_arcs() const {
    int count = 0;
    for (ArcIndex arc = 0; arc < active_non_sink_arc_count; ++arc) {
        if (arc_cap.at(arc) == 0) {
            count++;
        }
    }
    const ArcIndex sink_begin = static_cast<ArcIndex>(num_edges_before_sink);
    const ArcIndex sink_end = static_cast<ArcIndex>(arc_tail.size());
    for (ArcIndex arc = sink_begin; arc < sink_end; ++arc) {
        if (arc_cap.at(arc) == 0) {
            count++;
        }
    }
    return count;
}

FlowNetworkSnapshot HybridFlowPlanner::current_network_snapshot() const {
    FlowNetworkSnapshot snapshot;
    snapshot.time_horizon = current_makespan;
    snapshot.nodes = static_cast<int>(active_node_count);
    snapshot.arcs = static_cast<int>(
            active_non_sink_arc_count +
            (static_cast<ArcIndex>(arc_tail.size()) - static_cast<ArcIndex>(num_edges_before_sink)));
    snapshot.reached_locations = static_cast<int>(reached_locations.size());
    snapshot.map_edge_gadgets = 0;
    snapshot.blocked_arcs = count_blocked_arcs();
    snapshot.forced_commit_edges = static_cast<int>(commit_edges_d_to_s.size());
    return snapshot;
}

void HybridFlowPlanner::print_assigned_reservations() const {
    std::cout << "Blocked assigned reservations in direct network:" << std::endl;
    if (blocked_assigned_reservations.empty()) {
        std::cout << "  none" << std::endl;
        return;
    }
    for (const auto &reservation : blocked_assigned_reservations) {
        std::cout << "  arc " << reservation.arc << ": " << reservation.conflict << std::endl;
    }
}

void HybridFlowPlanner::print_minimum_conflict_cut_realization_constraints() const {
    std::cout << "Realization conflicts on the selected minimum-conflict minimum cut:"
              << std::endl;
    if (minimum_conflict_cut_realization_constraints.empty()) {
        std::cout << "  none" << std::endl;
        return;
    }
    for (const RealizationConflict &constraint : minimum_conflict_cut_realization_constraints) {
        std::cout << "  " << constraint << std::endl;
    }
}
