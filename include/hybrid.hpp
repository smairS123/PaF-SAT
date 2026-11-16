#ifndef HYBRID_SOLVER_HPP
#define HYBRID_SOLVER_HPP

#include "nrf_backend.hpp"
#include <iosfwd>
#include <string>

namespace hybrid {

inline constexpr const char *algorithm_name = "PaF-SAT";

struct Cell {
    int row = 0;
    int column = 0;
};

struct Problem {
    int rows = 0;
    int columns = 0;
    std::vector<std::string> grid;
    std::vector<Cell> vertex_cells;
    std::vector<std::vector<int>> cell_vertices;
    GraphProblem assigned;
    std::vector<int> unassigned_starts;
};

Problem read_problem(const std::string &map_path, const std::string &scenario_path);
int shortest_path_lower_bound(const Problem &problem);
void validate_paths(const Problem &problem, const Paths &assigned, const Paths &unassigned,
                    int horizon);

struct Options {
    double time_limit = 30.0;
    int max_makespan = -1; // No artificial horizon cap by default.
    std::ostream *trace = nullptr;
};

struct Result {
    std::string status = "unknown";
    std::string reason;
    int makespan = -1;
    int last_horizon = -1;
    int horizons_attempted = 0;
    int horizons_proven_unsat = 0;
    int last_failed_prefix = -1;
    int last_cut_capacity = -1;
    int corridor_solved = 0;   // 1 if the solution came from a corridor level
    std::uint64_t candidates = 0;
    std::uint64_t flow_checks = 0;
    std::uint64_t flow_failures = 0;
    std::uint64_t cut_clauses = 0;
    std::uint64_t cut_events = 0;
    std::uint64_t cut_literals = 0;
    double runtime_seconds = 0.0;
    double flow_seconds = 0.0;
    double clause_seconds = 0.0;
    NrfStats nrf;
    Paths assigned_paths;
    Paths unassigned_paths;
};

Result solve(const Problem &problem, const Options &options);
void write_result_csv(const std::string &path, const Problem &problem, const Result &result);
void write_solution_csv(const std::string &path, const Problem &problem, const Result &result);

} // namespace hybrid
#endif
