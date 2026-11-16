#ifndef HYBRID_FLOW_PLANNER_HPP
#define HYBRID_FLOW_PLANNER_HPP

#include <map>
#include <memory>
#include <sstream>
#include <cstdint>
#include "NATCBSDirectEarliestFailMinRC/earliest_fail_minrc_direct_flow_node.hpp"
#include "constraints/constraints.hpp"
#include "NATCBS/realization_conflict.hpp"
#include "solver_stats.hpp"
#if STRIP_OWN_WITH_ORTOOLS
#include "ortools/graph/graph.h"
#include "ortools/graph/max_flow.h"
#include "ortools/graph/min_cost_flow.h"
#endif

// Optimized copy of the review planner (see README.md of nrf_sat_flow_optimized):
//   step 1: arrays instead of hash maps for node and arc lookup;
//   step 2: one OR-Tools graph per horizon T, with sink arcs from every layer; a probe only switches which
//           layer's sink arcs are open, reservations are applied once per check, and Min-RC uses the same graph;
//   step 3: the maximum flow of the last failing probe (= t*) is kept for Min-RC instead of solved again
//           (UA_FLOW_REUSE_TSTAR=0 restores the second solve).
// The cuts, and therefore the whole search, are identical to the original planner.
class HybridFlowPlanner {
public:
    enum Result {
        SUCCESS,
        CONFLICT,
        INFEASIBLE,
    };

private:
    int map_rows;
    int map_cols;
    vector<vector<CellType>> map;
    int num_of_agents;
    int num_assigned_agents;
    vector<Location> agent_start_locations;
    bool verbose;
    SolverStats *stats;
    bool use_min_cost_flow;

    typedef int32_t NodeIndex;
    typedef int32_t ArcIndex;
#if STRIP_OWN_WITH_ORTOOLS
    typedef util::ReverseArcStaticGraph<NodeIndex, ArcIndex> Graph;
    typedef operations_research::GenericMinCostFlow<Graph> MinCostFlow;
    typedef operations_research::GenericMaxFlow<Graph> MaxFlow;
#endif
    typedef int32_t FlowQuantity;
    typedef int32_t CostValue;
    typedef int64_t LexicographicCapacity;

    struct BlockedAssignedReservation {
        RealizationConflict conflict;
        ArcIndex arc;
    };

    constexpr static const int dx[4] = {0, 1, 0, -1};
    constexpr static const int dy[4] = {1, 0, -1, 0};
    constexpr static const FlowQuantity CAP = 1;
    constexpr static const CostValue NO_COST = 0;
    constexpr static const CostValue UNIT_COST = 1;

    vector<EarliestFailMinRCDirectFlowNode> idx_to_node;
    // Step 1 (arrays instead of hash maps). Dense node lookup:
    // layer_nodes[time][(x * map_cols + y) * 2 + (IN ? 1 : 0)], -1 when the node does not exist.
    vector<vector<NodeIndex>> layer_nodes;
    // Non-sink arcs leaving each node (replaces the nested hash map tail -> head -> arc).
    // Declared before source_idx because the constructor creates the source/sink nodes first.
    vector<vector<ArcIndex>> out_arcs;
    EarliestFailMinRCDirectFlowNode source_node;
    NodeIndex source_idx;
    EarliestFailMinRCDirectFlowNode sink_node;
    NodeIndex sink_idx;
    int current_makespan;
    int max_makespan;
    int num_edges_before_sink;
    NodeIndex active_node_count;
    ArcIndex active_non_sink_arc_count;

    vector<NodeIndex> arc_tail;
    vector<NodeIndex> arc_head;
    vector<CostValue> arc_cost;
    vector<FlowQuantity> arc_cap;
#if STRIP_OWN_WITH_ORTOOLS
    vector<ArcIndex> arc_permutation;
#endif
    vector<FlowQuantity> arc_flow;
    FlowQuantity maximum_flow;
    CostValue optimal_cost;

    vector<Location> reached_locations;
    unordered_set<Location> new_reached_locations;
    unordered_set<Location> recently_reached_locations;
    vector<std::pair<Location, Location>> map_edges;
    bool reached_all = false;

    unordered_map<NodeIndex, NodeIndex> commit_edges_d_to_s;
    vector<ArcIndex> deleted_arcs;
    vector<BlockedAssignedReservation> blocked_assigned_reservations;
    vector<RealizationConflict> minimum_conflict_cut_realization_constraints;

    vector<NodeIndex> makespan_to_node_count;          // indexed by makespan
    vector<ArcIndex> makespan_to_non_sink_arc_count;   // indexed by makespan

    // Step 2: the OR-Tools graph of horizon graph_horizon. Every layer t has its own small sink node
    // L_t = layer_sink_base + t, so the real sink only has T + 2 arcs (one big sink node with the sink
    // arcs of every layer made the min-cost flow several times slower on large maps).
    // Arc order before OR-Tools' permutation:
    //   [0, graph_non_sink_arc_count)                  the non-sink arcs (same indices as arc_tail/arc_head)
    //   [sink_arc_begin[t], sink_arc_begin[t + 1])     OUT(t, v) -> L_t for each cell v of layer t, t = 0..T
    //   layer_sink_arc_base + t                        L_t -> sink: capacity K opens layer t, 0 closes it
    //   total_source_arc, total_sink_arc               total_source -> source and sink -> total_sink
    int graph_horizon = -1;
#if STRIP_OWN_WITH_ORTOOLS
    std::unique_ptr<Graph> flow_graph;
#endif
    NodeIndex layer_sink_base = -1;
    NodeIndex total_source = -1;
    NodeIndex total_sink = -1;
    NodeIndex graph_node_count = 0;
    ArcIndex graph_non_sink_arc_count = 0;
    vector<ArcIndex> sink_arc_begin;
    vector<NodeIndex> sink_arc_tails;
    ArcIndex layer_sink_arc_base = -1;
    ArcIndex total_source_arc = -1;
    ArcIndex total_sink_arc = -1;
    // Two ordinary max-flow solvers on that graph (step 3 keeps the failing probe of t* in one of them)
    // and one for the weighted Min-RC network. Solver 1 and the weighted solver are created only at the
    // first failed check of a horizon. Each solver keeps its own capacities; we only change the arcs that
    // differ from what it had: the blocked reservation arcs and the open layer.
#if STRIP_OWN_WITH_ORTOOLS
    std::unique_ptr<MaxFlow> ordinary_flow[2];
    int solver_open_layer[2] = {-1, -1};
    long long solver_check[2] = {-1, -1};
    vector<ArcIndex> solver_blocked[2];
    std::unique_ptr<MaxFlow> weighted_flow;
#endif
    long long check_counter = 0;
    bool reuse_earliest_failing_probe = true;
    // Warm start (this version): test the previous check's t* first (UA_FLOW_WARM_TSTAR, default on).
    bool warm_start_tstar = true;
    int previous_tstar = -1;
    bool use_earliest_failing_prefix = true;
    bool use_minimum_conflict_cut = true;

    int node_slot(EarliestFailMinRCDirectFlowNode::Type type, const Location &location) const {
        return (location.x * map_cols + location.y) * 2 + (type == EarliestFailMinRCDirectFlowNode::IN ? 1 : 0);
    }
    NodeIndex find_node(EarliestFailMinRCDirectFlowNode::Type type, int time, const Location &location) const;
    NodeIndex at_node(EarliestFailMinRCDirectFlowNode::Type type, int time, const Location &location) const;
    NodeIndex get_node_idx(const EarliestFailMinRCDirectFlowNode &node);
    ArcIndex get_arc_idx(NodeIndex tail, NodeIndex head) const;
    bool has_arc(NodeIndex tail, NodeIndex head) const;
    void add_arc(NodeIndex tail, NodeIndex head, CostValue cost = NO_COST);
    void update_map_edges();
    void prepare_horizon(int makespan);
    void build_flow_graph(int makespan);
    void set_active_prefix(int makespan);
#if STRIP_OWN_WITH_ORTOOLS
    MaxFlow &ordinary_solver(int solver);
    MaxFlow &weighted_solver();
    // This version: the OR-Tools graph is built lazily, only when an OR-Tools solver is requested.
    bool ortools_graph_ready = false;
    void ensure_ortools_graph();
#endif
    ArcIndex layer_sink_arc(int layer) const { return layer_sink_arc_base + layer; }

    void apply_assigned_reservations(const vector<shared_ptr<TimedPath>> &paths_to_realize);
    void block_vertex_reservation(int assigned_id, const Location &location, int time);
    void block_edge_reservation(int assigned_id, const Location &from, const Location &to, int time);
    bool block_arc(NodeIndex tail, NodeIndex head, ArcIndex *blocked_arc = nullptr);
    Result solve_prefix(int solver, int prefix);
#if STRIP_OWN_WITH_ORTOOLS
    Result solve_prefix_ortools(int solver, int prefix);
#endif
    // Incremental max-flow (this version): per prefix, the unit flow of the previous check is kept and only
    // repaired: paths through newly blocked arcs are cancelled, then BFS augmenting paths are added.
    // The maximum-flow value and the residual source side are the same for every maximum flow, so the
    // search (yes/no answers and cuts) is identical to solving from scratch.
    struct IncrementalState {
        long long check = -1;
        int value = 0;
        std::vector<uint8_t> flow;
        std::vector<NodeIndex> source_side;
    };
    bool use_incremental_flow = true;      // UA_FLOW_INCREMENTAL (default on)
    bool incremental_verify = false;       // UA_FLOW_INCREMENTAL_VERIFY: compare every answer with OR-Tools
    std::map<int, IncrementalState> incremental_states;
    std::vector<ArcIndex> in_begin;   // CSR of incoming non-sink arcs, built only when a repair is needed
    std::vector<ArcIndex> in_list;
    std::size_t in_arcs_built_for = 0;
    std::vector<int> inc_mark;
    int inc_stamp = 0;
    std::vector<ArcIndex> inc_parent;
    std::vector<char> inc_parent_forward;
    std::vector<NodeIndex> inc_queue;
    Result solve_prefix_incremental(int solver, int prefix);
    void ensure_in_arcs();
    // Strip Min-RC (this version): area A (source side of the failed unit flow) becomes one source,
    // area B (nodes that can still reach a free end point) one sink; only the strip between them is searched.
    std::vector<char> strip_region;                 // 0 = strip, 1 = area A, 2 = area B
    std::vector<LexicographicCapacity> strip_delta; // weighted flow on top of M * unit flow
    std::vector<ArcIndex> strip_touched;
    std::vector<int> strip_level;
    std::vector<int> strip_stamp;
    std::vector<std::size_t> strip_ptr;
    std::vector<ArcIndex> strip_exits;
    std::vector<NodeIndex> strip_queue;
    int strip_phase = 0;
    int strip_sink_level = -1;
    LexicographicCapacity strip_minrc_flow(const std::vector<LexicographicCapacity> &penalty,
                                           LexicographicCapacity multiplier, std::vector<NodeIndex> &source_side);
    LexicographicCapacity strip_dfs(NodeIndex node, LexicographicCapacity limit, ArcIndex arcs,
                                    const std::vector<LexicographicCapacity> &penalty,
                                    LexicographicCapacity multiplier, const IncrementalState &state);
    // Own Min-RC weighted flow (this version): starts from the kept unit flow times M, then Dinic phases.
    bool use_own_minrc = true;             // UA_FLOW_OWN_MINRC (default on; needs the incremental flow)
    std::vector<LexicographicCapacity> own_w;
    std::vector<int> own_level;
    std::vector<int> own_level_stamp;
    std::vector<std::size_t> own_ptr;
    int own_phase = 0;
    LexicographicCapacity own_minrc_flow(const std::vector<LexicographicCapacity> &penalty,
                                         LexicographicCapacity multiplier, std::vector<NodeIndex> &source_side);
    LexicographicCapacity own_minrc_dfs(NodeIndex node, LexicographicCapacity limit, int prefix, ArcIndex arcs,
                                        const std::vector<LexicographicCapacity> &penalty,
                                        LexicographicCapacity multiplier);
    bool is_prefix_terminal(NodeIndex node, int prefix) const;
    void incremental_cancel(IncrementalState &state, ArcIndex arc, int prefix);
    bool incremental_augment(IncrementalState &state, int prefix);
    // This version: depth-first augmenting paths (UA_FLOW_AUGMENT=bfs restores the BFS of the strip version).
    bool use_dfs_augment = true;
    std::vector<std::size_t> dfs_cursor;
    std::vector<NodeIndex> dfs_stack;
    bool incremental_augment_dfs(IncrementalState &state, int prefix);
    const std::vector<NodeIndex> &incremental_source_side(int prefix) const;
    void select_ordinary_minimum_cut(int solver);
    void select_minimum_conflict_cut(int solver);
    void compute_selected_flow(int solver);
    FlowQuantity required_flow() const;
#if STRIP_OWN_WITH_ORTOOLS
    ArcIndex permuted_arc(ArcIndex arc) const;
#endif
    void normalize_anonymous_swaps();
    NodeIndex consume_next_flow_arc(NodeIndex tail);
    void consume_flow_arc(NodeIndex tail, NodeIndex head);
    void print_selected_flow() const;
    void print_network_summary(const string &stage) const;
    void print_assigned_reservations() const;
    void print_minimum_conflict_cut_realization_constraints() const;
    void update_minimum_conflict_cut_realization_constraints(
            const vector<NodeIndex> &source_side_nodes,
            NodeIndex augmented_node_count);
    int count_blocked_arcs() const;
    FlowNetworkSnapshot current_network_snapshot() const;

public:
    vector<shared_ptr<TimedPath>> agent_paths;
    shared_ptr<RealizationConflict> realization_conflict;

    HybridFlowPlanner(int map_rows,
                         int map_cols,
                         const vector<vector<CellType>> &map,
                         vector<Location> &agent_start_locations,
                         int num_assigned_agents,
                         bool verbose,
                         SolverStats *stats = nullptr,
                         bool use_min_cost_flow = false);

    Result find_paths(const vector<shared_ptr<TimedPath>> &paths_to_realize,
                      const vector<shared_ptr<Constraints>> &constraints_table,
                      int makespan);
    void extract_agent_paths();
    const vector<RealizationConflict> &get_minimum_conflict_cut_realization_constraints() const;
    int active_horizon() const { return current_makespan; }
    int achieved_flow() const { return maximum_flow; }
};

#endif // HYBRID_FLOW_PLANNER_HPP
