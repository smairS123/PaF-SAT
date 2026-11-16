#ifndef UA_MAPF_REALIZATION_CONFLICT_HPP
#define UA_MAPF_REALIZATION_CONFLICT_HPP

#include "../common.hpp"

struct RealizationConflict{
    enum class Type {
        Vertex,
        Edge,
    };

    Type type;
    int assigned_id;
    Location location1;
    Location location2;
    int time;

    RealizationConflict(int assigned_id, Location location, int time) :
            type(Type::Vertex), assigned_id(assigned_id), location1(location), location2(), time(time) {}

    RealizationConflict(int assigned_id, Location location1, Location location2, int time) :
            type(Type::Edge), assigned_id(assigned_id), location1(location1), location2(location2), time(time) {}

    friend std::ostream &operator<<(std::ostream &os, const RealizationConflict &c) {
        os << "RealizationConflict: assigned: " << c.assigned_id;
        if (c.type == Type::Vertex) {
            return os << " vertex: " << c.location1 << " time: " << c.time;
        }
        return os << " edge: " << c.location1 << "->" << c.location2 << " time: " << c.time;
    }
};

#endif //UA_MAPF_REALIZATION_CONFLICT_HPP
