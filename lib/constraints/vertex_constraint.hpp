#ifndef UA_MAPF_VERTEX_CONSTRAINT_HPP
#define UA_MAPF_VERTEX_CONSTRAINT_HPP

#include "../common.hpp"

struct VertexConstraint {
    int time;
    Location location;

    VertexConstraint(int time, Location location);

    bool operator<(const VertexConstraint& other) const;
    bool operator==(const VertexConstraint& other) const;
    friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c);
    inline friend size_t hash_value(VertexConstraint const& c);
};


#endif //UA_MAPF_VERTEX_CONSTRAINT_HPP
