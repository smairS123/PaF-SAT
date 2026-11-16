#include "hybrid.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>

namespace hybrid {
namespace {

std::string first_nonempty_line(std::istream &stream) {
    std::string line;
    while (std::getline(stream, line)) {
        if (line.find_first_not_of(" \t\r\n") != std::string::npos) return line;
    }
    throw std::invalid_argument("Unexpected end of input");
}

std::vector<int> counts(const std::string &line) {
    std::istringstream stream(line);
    std::vector<int> result;
    int value;
    while (stream >> value) result.push_back(value);
    if (!stream.eof()) throw std::invalid_argument("Non-integer value in input header");
    return result;
}

} // namespace

Problem read_problem(const std::string &map_path, const std::string &scenario_path) {
    std::ifstream map_file(map_path), scenario_file(scenario_path);
    if (!map_file) throw std::invalid_argument("Cannot open map: " + map_path);
    if (!scenario_file) throw std::invalid_argument("Cannot open scenario: " + scenario_path);
    Problem result;
    const std::string header = first_nonempty_line(map_file);
    std::istringstream header_stream(header);
    std::string first;
    header_stream >> first;
    const bool movingai_map = first == "type";
    if (movingai_map) {
        std::string key;
        bool found_grid = false;
        while (map_file >> key) {
            if (key == "height") map_file >> result.rows;
            else if (key == "width") map_file >> result.columns;
            else if (key == "map") { found_grid = true; break; }
            else throw std::invalid_argument("Unexpected MovingAI map header field: " + key);
        }
        if (!found_grid) throw std::invalid_argument("MovingAI map has no grid");
    } else {
        const auto dimensions = counts(header);
        if (dimensions.size() != 2) throw std::invalid_argument("Expected map rows and columns");
        result.rows = dimensions[0];
        result.columns = dimensions[1];
    }
    if (result.rows <= 0 || result.columns <= 0 ||
        static_cast<long long>(result.rows) * result.columns > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("Invalid map dimensions");
    }
    result.cell_vertices.assign(result.rows, std::vector<int>(result.columns, -1));
    for (int row = 0; row < result.rows; ++row) {
        std::string cells;
        if (!(map_file >> cells) || cells.size() != static_cast<std::size_t>(result.columns)) {
            throw std::invalid_argument("Incorrect map row length");
        }
        for (int col = 0; col < result.columns; ++col) {
            const bool free = movingai_map ? (cells[col] == '.' || cells[col] == 'G' || cells[col] == 'S')
                                         : (cells[col] != '#' && cells[col] != '@');
            cells[col] = free ? '.' : '@';
            if (free) {
                result.cell_vertices[row][col] = static_cast<int>(result.vertex_cells.size());
                result.vertex_cells.push_back({row, col});
            }
        }
        result.grid.push_back(cells);
    }
    result.assigned.vertices = static_cast<int>(result.vertex_cells.size());
    if (!result.assigned.vertices) throw std::invalid_argument("Map has no traversable vertices");
    for (int v = 0; v < result.assigned.vertices; ++v) {
        const Cell cell = result.vertex_cells[v];
        for (const auto &[dr, dc] : {std::pair<int, int>{1, 0}, {0, 1}}) {
            const int row = cell.row + dr, col = cell.column + dc;
            if (row < result.rows && col < result.columns && result.cell_vertices[row][col] >= 0) {
                result.assigned.edges.emplace_back(v, result.cell_vertices[row][col]);
            }
        }
    }

    const auto sizes = counts(first_nonempty_line(scenario_file));
    if (sizes.size() != 2 && sizes.size() != 3) {
        throw std::invalid_argument("Use a native scenario: A U, or legacy U A tasks; not raw MovingAI .scen");
    }
    const bool legacy = sizes.size() == 3;
    const int assigned = sizes[legacy ? 1 : 0], unassigned = sizes[legacy ? 0 : 1];
    if (assigned < 0 || unassigned < 0 ||
        static_cast<long long>(assigned) + unassigned > result.assigned.vertices ||
        (legacy && (sizes[2] < 0 || sizes[2] > assigned))) {
        throw std::invalid_argument("Invalid agent counts");
    }
    auto read_vertex = [&]() {
        int x, y;
        if (!(scenario_file >> x >> y)) throw std::invalid_argument("Missing scenario coordinates");
        // Native and MovingAI text use x=column, y=row. The flow code's
        // Location.x is row; keep that conversion at the adapter boundary.
        if (x < 0 || y < 0 || x >= result.columns || y >= result.rows || result.cell_vertices[y][x] < 0) {
            throw std::invalid_argument("Scenario coordinate is outside the map or blocked");
        }
        return result.cell_vertices[y][x];
    };
    auto read_unassigned = [&]() {
        for (int a = 0; a < unassigned; ++a) result.unassigned_starts.push_back(read_vertex());
    };
    if (legacy) read_unassigned();
    int tasks = 0;
    for (int a = 0; a < assigned; ++a) {
        const int start = read_vertex(), goal = read_vertex();
        result.assigned.starts.push_back(start);
        result.assigned.goals.push_back(goal);
        tasks += start != goal;
    }
    if (!legacy) read_unassigned();
    if (legacy && tasks != sizes[2]) throw std::invalid_argument("Incorrect legacy task count");
    std::string extra;
    if (scenario_file >> extra) throw std::invalid_argument("Unexpected data after scenario agents");
    std::set<int> starts;
    for (int start : result.assigned.starts) {
        if (!starts.insert(start).second) throw std::invalid_argument("Agents share a start vertex");
    }
    for (int start : result.unassigned_starts) {
        if (!starts.insert(start).second) throw std::invalid_argument("Agents share a start vertex");
    }
    return result;
}

int shortest_path_lower_bound(const Problem &problem) {
    const auto &graph = problem.assigned;
    if (std::set<int>(graph.goals.begin(), graph.goals.end()).size() != graph.goals.size()) return -1;
    std::vector<std::vector<int>> adjacent(graph.vertices);
    for (const auto &[a, b] : graph.edges) {
        adjacent[a].push_back(b);
        adjacent[b].push_back(a);
    }
    int bound = 0;
    for (std::size_t a = 0; a < graph.starts.size(); ++a) {
        std::vector<int> distance(graph.vertices, -1);
        std::queue<int> open;
        open.push(graph.starts[a]);
        distance[graph.starts[a]] = 0;
        while (!open.empty() && distance[graph.goals[a]] < 0) {
            const int v = open.front();
            open.pop();
            for (int next : adjacent[v]) {
                if (distance[next] < 0) {
                    distance[next] = distance[v] + 1;
                    open.push(next);
                }
            }
        }
        if (distance[graph.goals[a]] < 0) return -1;
        bound = std::max(bound, distance[graph.goals[a]]);
    }
    return bound;
}

void validate_paths(const Problem &problem, const Paths &assigned, const Paths &unassigned, int horizon) {
    if (horizon < 0 || assigned.size() != problem.assigned.starts.size() ||
        unassigned.size() != problem.unassigned_starts.size()) {
        throw std::runtime_error("Solution has an invalid horizon or agent count");
    }
    std::vector<const std::vector<int> *> paths;
    auto check = [&](const Paths &part, const std::vector<int> &starts, bool has_goal) {
        for (std::size_t a = 0; a < part.size(); ++a) {
            const auto &path = part[a];
            if (path.size() != static_cast<std::size_t>(horizon) + 1 || path.front() != starts[a] ||
                (has_goal && path.back() != problem.assigned.goals[a])) {
                throw std::runtime_error("Solution has an incorrect start, endpoint, or path length");
            }
            for (int t = 0; t <= horizon; ++t) {
                if (path[t] < 0 || path[t] >= problem.assigned.vertices) {
                    throw std::runtime_error("Solution visits an invalid vertex");
                }
                if (t > 0) {
                    const auto from = problem.vertex_cells[path[t - 1]], to = problem.vertex_cells[path[t]];
                    if (std::abs(from.row - to.row) + std::abs(from.column - to.column) > 1) {
                        throw std::runtime_error("Solution contains a non-adjacent move");
                    }
                }
            }
            paths.push_back(&path);
        }
    };
    check(assigned, problem.assigned.starts, true);
    check(unassigned, problem.unassigned_starts, false);
    for (int t = 0; t <= horizon; ++t) {
        std::vector<int> owner(problem.assigned.vertices, -1);
        for (std::size_t a = 0; a < paths.size(); ++a) {
            const int v = paths[a]->at(t);
            if (owner[v] >= 0) throw std::runtime_error("Solution contains a vertex collision");
            owner[v] = static_cast<int>(a);
        }
        if (t == 0) continue;
        for (std::size_t a = 0; a < paths.size(); ++a) {
            const int previous = paths[a]->at(t - 1), current = paths[a]->at(t);
            const int b = owner[previous];
            if (previous != current && b >= 0 && paths[b]->at(t - 1) == current) {
                throw std::runtime_error("Solution contains an opposite-edge swap");
            }
        }
    }
}

} // namespace hybrid
