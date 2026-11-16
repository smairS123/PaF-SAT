#include "hybrid.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace hybrid {
namespace {

void prepare_parent(const std::string &path) {
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
}

std::string csv_text(const std::string &text) {
    std::string escaped = "\"";
    for (char c : text) { if (c == '"') escaped += '"'; escaped += c; }
    return escaped + '"';
}

} // namespace

void write_result_csv(const std::string &path, const Problem &problem, const Result &r) {
    const std::string header = "algorithm,status,success,timeout,makespan,runtime_ms,assigned_count,unassigned_count,"
        "last_horizon,horizons_attempted,horizons_proven_unsat,assigned_candidates,sat_calls,assigned_refinements,"
        "assigned_collision_events,flow_checks,flow_failures,cut_clauses,realization_conflicts,cut_literals,"
        "encoding_ms,sat_ms,decode_check_ms,flow_ms,clause_ms,last_failed_prefix,last_cut_capacity,kept_collisions,kept_cuts,corridor_sessions,corridor_solved,reason";
    prepare_parent(path);
    bool empty = true;
    if (std::filesystem::exists(path) && std::filesystem::file_size(path) > 0) {
        std::ifstream previous(path);
        std::string previous_header;
        std::getline(previous, previous_header);
        if (previous_header != header) throw std::runtime_error("Refusing to append to an incompatible results CSV");
        empty = false;
    }
    std::ofstream out(path, std::ios::app);
    if (!out) throw std::runtime_error("Cannot write results: " + path);
    if (empty) out << header << '\n';
    out << std::setprecision(12) << algorithm_name << ',' << r.status << ',' << (r.status == "solved")
        << ',' << (r.status == "timeout") << ',';
    if (r.status == "solved") out << r.makespan;
    out << ',' << r.runtime_seconds * 1000 << ',' << problem.assigned.starts.size() << ','
        << problem.unassigned_starts.size() << ',' << r.last_horizon << ',' << r.horizons_attempted << ','
        << r.horizons_proven_unsat << ',' << r.candidates << ',' << r.nrf.sat_calls << ','
        << r.nrf.assigned_refinements << ',' << r.nrf.assigned_collision_events << ','
        << r.flow_checks << ',' << r.flow_failures << ',' << r.cut_clauses << ',' << r.cut_events << ','
        << r.cut_literals << ',' << r.nrf.encoding_seconds * 1000 << ',' << r.nrf.sat_seconds * 1000 << ','
        << r.nrf.decode_check_seconds * 1000 << ',' << r.flow_seconds * 1000 << ',' << r.clause_seconds * 1000
        << ',' << r.last_failed_prefix << ',' << r.last_cut_capacity << ','
        << r.nrf.kept_collisions << ',' << r.nrf.kept_cuts << ','
        << r.nrf.corridor_sessions << ',' << r.corridor_solved << ',' << csv_text(r.reason) << '\n';
    if (!out) throw std::runtime_error("Results CSV write failed");
}

void write_solution_csv(const std::string &path, const Problem &problem, const Result &r) {
    if (r.status != "solved") throw std::logic_error("Cannot write a solution for an unsuccessful run");
    prepare_parent(path);
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Cannot write solution: " + path);
    out << "role,agent,time,x,y\n";
    auto write = [&](const Paths &paths, const char *role) {
        for (std::size_t a = 0; a < paths.size(); ++a) {
            for (std::size_t t = 0; t < paths[a].size(); ++t) {
                const auto cell = problem.vertex_cells.at(paths[a][t]);
                out << role << ',' << a << ',' << t << ',' << cell.column << ',' << cell.row << '\n';
            }
        }
    };
    write(r.assigned_paths, "assigned");
    write(r.unassigned_paths, "unassigned");
    if (!out) throw std::runtime_error("Solution CSV write failed");
}

} // namespace hybrid
