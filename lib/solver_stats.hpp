#ifndef UA_MAPF_SOLVER_STATS_HPP
#define UA_MAPF_SOLVER_STATS_HPP

#include <algorithm>
#include <string>

struct FlowNetworkSnapshot {
    int time_horizon = 0;
    int nodes = 0;
    int arcs = 0;
    int reached_locations = 0;
    int map_edge_gadgets = 0;
    int blocked_arcs = 0;
    int forced_commit_edges = 0;
};

struct SolverStats {
    std::string algorithm;
    std::string priority_heuristic = "none";
    std::string priority_order = "-";
    std::string instance_name;
    std::string map_name;
    std::string scenario_name;
    std::string outcome = "unknown";
    std::string failure_reason = "";

    int map_rows = 0;
    int map_cols = 0;
    int assigned_count = 0;
    int unassigned_count = 0;

    int makespan = -1;
    int sst = -1;
    double runtime_seconds = 0.0;
    bool timeout = false;

    double assigned_planning_seconds = 0.0;
    double flow_total_seconds = 0.0;
    double flow_network_build_seconds = 0.0;
    double flow_sources_sinks_seconds = 0.0;
    double flow_reservations_seconds = 0.0;
    double flow_ortools_build_seconds = 0.0;
    double flow_max_flow_seconds = 0.0;
    double flow_extract_paths_seconds = 0.0;
    double cut_bfs_seconds = 0.0;

    int flow_checks = 0;
    int flow_successes = 0;
    int flow_failures = 0;
    int flow_infeasible = 0;
    int flow_conflicts = 0;
    int last_flow_nodes = 0;
    int last_flow_arcs = 0;
    int last_flow_blocked_arcs = 0;
    int max_flow_nodes = 0;
    int max_flow_arcs = 0;
    int max_flow_blocked_arcs = 0;

    int cut_bfs_calls = 0;
    int cut_bfs_complete_calls = 0;
    int cut_bfs_limited_calls = 0;
    int cut_bfs_improved_calls = 0;
    long long cut_bfs_states = 0;
    long long cut_bfs_baseline_conflicts = 0;
    long long cut_bfs_selected_conflicts = 0;
    long long cut_bfs_conflict_reduction = 0;

    int cbs_nodes_created = 0;
    int cbs_nodes_expanded = 0;
    int assigned_conflicts = 0;
    int assigned_replans = 0;
    int realization_conflicts = 0;
    int adaptive_switches = 0;
    int adaptive_individual_failure_nodes = 0;
    int adaptive_individual_children = 0;
    int adaptive_max_stagnation_streak = 0;
    int group_parts = 0;
    int individual_every_depth = 0;
    unsigned int group_partition_seed = 0;
    int partitioned_failure_nodes = 0;
    int periodic_individual_failure_nodes = 0;
    int partitioned_children = 0;
    int periodic_individual_children = 0;

    int pbs_iterations = 0;
    int pbs_repair_attempts = 0;
    int pbs_repairs_applied = 0;
    int pbs_failed_repair_candidates = 0;
    int pbs_suffix_repair_attempts = 0;
    int pbs_successful_suffix_repairs = 0;
    int pbs_repair_budget = 0;
    int pbs_repair_budget_used = 0;

    void record_flow_snapshot(const FlowNetworkSnapshot &snapshot) {
        last_flow_nodes = snapshot.nodes;
        last_flow_arcs = snapshot.arcs;
        last_flow_blocked_arcs = snapshot.blocked_arcs;
        max_flow_nodes = std::max(max_flow_nodes, snapshot.nodes);
        max_flow_arcs = std::max(max_flow_arcs, snapshot.arcs);
        max_flow_blocked_arcs = std::max(max_flow_blocked_arcs, snapshot.blocked_arcs);
    }
};

#endif //UA_MAPF_SOLVER_STATS_HPP
