#ifndef HYBRID_NRF_BACKEND_HPP
#define HYBRID_NRF_BACKEND_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace hybrid {

using Clock = std::chrono::steady_clock;
using Paths = std::vector<std::vector<int>>;

struct GraphProblem {
    int vertices = 0;
    std::vector<std::pair<int, int>> edges;
    std::vector<int> starts;
    std::vector<int> goals;
};

struct Occupancy {
    int agent = 0; // Zero-based outside boOX.
    int vertex = 0;
    int time = 0;
    bool operator<(const Occupancy &other) const {
        return std::tie(agent, vertex, time) < std::tie(other.agent, other.vertex, other.time);
    }
    bool operator==(const Occupancy &other) const {
        return agent == other.agent && vertex == other.vertex && time == other.time;
    }
};

// One clause: OR of the negations of these occupancy variables.
using CutClause = std::vector<Occupancy>;

struct NrfStats {
    std::uint64_t sat_calls = 0;
    std::uint64_t assigned_refinements = 0;
    std::uint64_t assigned_collision_events = 0;
    std::uint64_t kept_collisions = 0;  // keep: collision clauses re-added from earlier makespans (all horizons)
    std::uint64_t kept_cuts = 0;        // keep: flow cut clauses re-added from earlier makespans (all horizons)
    std::uint64_t corridor_sessions = 0; // sessions started at a corridor level (smaller MDDs)
    double encoding_seconds = 0.0;
    double sat_seconds = 0.0;
    double decode_check_seconds = 0.0;
};

enum class SatStatus { Candidate, Unsat, Timeout };

// Keep: the clauses learned at earlier makespans, given to the session of the next makespan.
// boOX does the same for its collisions (its Context lives across the makespan/cost loop); here the flow cut
// clauses are kept too. Every kept clause stays true at a larger makespan T' > T:
//   - a collision clause only says "two assigned agents are not in the same place at time t";
//   - a cut clause comes from a flow failure up to t* <= T, which depends only on the points blocked up to t*.
// The NRF variables of a kept clause exist again at T' (each agent's MDD layer t only grows with the
// makespan); a clause whose variable is missing is already satisfied and is skipped.
class KeptClauses {
public:
    KeptClauses();
    ~KeptClauses();
    KeptClauses(const KeptClauses &) = delete;
    KeptClauses &operator=(const KeptClauses &) = delete;
    std::size_t collisions() const;   // vertex and edge collisions kept so far
    std::size_t cuts() const;         // flow cut clauses kept so far
    struct Data;                      // defined in nrf_backend.cpp (keeps boOX types out of this header)
    Data &data() { return *data_; }

private:
    std::unique_ptr<Data> data_;
};

// Settings of one session that are new in this version. The defaults give exactly the earlier search.
struct SessionOptions {
    // Corridor: >= 0 keeps in each agent's MDD only the cells with ds(v) + dg(v) <= d + corridor_slack
    // (d = the agent's shortest start-goal distance). -1 = the full MDD. The corridor is used only when the full
    // MDD has at least `corridor_min_nodes` nodes (all agents, all layers); otherwise the session uses the full MDD.
    int corridor_slack = -1;
    std::uint64_t corridor_min_nodes = 0;
};

// Keeps Glucose, its learned clauses, and the NRF model alive at one horizon.
// The implementation deliberately does not expose boOX headers to native flow.
class NrfSession {
public:
    // The clauses in `kept` are added first, and every clause this session learns is added to `kept`.
    // kept == nullptr starts from an empty formula.
    NrfSession(const GraphProblem &problem, int horizon, Clock::time_point deadline,
               NrfStats &stats, KeptClauses *kept = nullptr, const SessionOptions &options = SessionOptions());
    ~NrfSession();
    NrfSession(const NrfSession &) = delete;
    NrfSession &operator=(const NrfSession &) = delete;
    SatStatus next(Paths &paths);
    void forbid_cut(const CutClause &clause);
    // True if this session uses the smaller corridor MDDs. UNSAT of such a session does NOT prove that the
    // horizon is impossible; only UNSAT of a session with corridor() == false does.
    bool corridor() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hybrid
#endif
