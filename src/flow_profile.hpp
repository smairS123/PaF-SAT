#pragma once

// Compile out all instrumentation unless the separate profiling build opts in.
#ifdef UA_FLOW_PROFILE
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <ostream>
#include <vector>

namespace flow_profile {
using Clock = std::chrono::steady_clock;
class Scope;
struct Event {
    std::uint64_t check;
    int horizon;
    const char *mode;
    int active_horizon;
    const char *stage;
    const char *kind;
    std::int64_t value;
    double seconds;
    double self_seconds;
};
struct Recorder {
    std::vector<Event> events;
    std::uint64_t check = 0;
    int horizon = -1;
    const char *mode = "init";
    int active_horizon = -1;
    Scope *top = nullptr;

    Recorder() { events.reserve(8192); }
    void write(std::ostream &out) const {
        out << "check,horizon,mode,active_horizon,stage,kind,value,seconds,self_seconds\n";
        out << std::setprecision(17);
        for (const auto &e : events) {
            out << e.check << ',' << e.horizon << ',' << e.mode << ',' << e.active_horizon
                << ',' << e.stage << ',' << e.kind << ',' << e.value << ','
                << e.seconds << ',' << e.self_seconds << '\n';
        }
    }
};
inline thread_local Recorder *active = nullptr;

class Session {
    Recorder *previous;
public:
    explicit Session(Recorder *recorder) : previous(active) { active = recorder; }
    ~Session() { active = previous; }
};

class Scope {
    Recorder *recorder = nullptr;
    Scope *parent = nullptr;
    Clock::time_point started;
    double children = 0;
    Event event{};
public:
    explicit Scope(const char *stage) { begin(stage); }
    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;
    ~Scope() { end(); }
    void begin(const char *stage) {
        recorder = active;
        if (!recorder) return;
        parent = recorder->top;
        recorder->top = this;
        children = 0;
        event = {recorder->check, recorder->horizon, recorder->mode,
                 recorder->active_horizon, stage, "time", 1, 0, 0};
        started = Clock::now();
    }
    void end() {
        if (!recorder) return;
        event.seconds = std::chrono::duration<double>(Clock::now() - started).count();
        event.self_seconds = event.seconds - children;
        if (parent) parent->children += event.seconds;
        recorder->top = parent;
        recorder->events.push_back(event);
        recorder = nullptr;
    }
    void next(const char *stage) { end(); begin(stage); }
};

inline void context(const char *mode, int horizon) {
    if (active) { active->mode = mode; active->active_horizon = horizon; }
}
inline void check(int horizon) {
    if (active) { ++active->check; active->horizon = horizon; context("full", horizon); }
}
inline void count(const char *name, std::int64_t value = 1) {
    if (active) active->events.push_back({active->check, active->horizon, active->mode,
        active->active_horizon, name, "count", value, 0, 0});
}
} // namespace flow_profile

#define FLOW_PROFILE_JOIN_(a, b) a##b
#define FLOW_PROFILE_JOIN(a, b) FLOW_PROFILE_JOIN_(a, b)
#define FLOW_PROFILE_SCOPE(stage) flow_profile::Scope FLOW_PROFILE_JOIN(profile_scope_, __LINE__)(stage)
#define FLOW_PROFILE_SEGMENT(name, stage) flow_profile::Scope name(stage)
#define FLOW_PROFILE_NEXT(name, stage) name.next(stage)
#define FLOW_PROFILE_END(name) name.end()
#define FLOW_PROFILE_CHECK(horizon) flow_profile::check(horizon)
#define FLOW_PROFILE_CONTEXT(mode, horizon) flow_profile::context(mode, horizon)
#define FLOW_PROFILE_COUNT(name, value) flow_profile::count(name, value)
#else
#define FLOW_PROFILE_SCOPE(stage) ((void)0)
#define FLOW_PROFILE_SEGMENT(name, stage) ((void)0)
#define FLOW_PROFILE_NEXT(name, stage) ((void)0)
#define FLOW_PROFILE_END(name) ((void)0)
#define FLOW_PROFILE_CHECK(horizon) ((void)0)
#define FLOW_PROFILE_CONTEXT(mode, horizon) ((void)0)
#define FLOW_PROFILE_COUNT(name, value) ((void)0)
#endif
