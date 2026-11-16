#ifndef UA_MAPF_EDGE_CONSTRAINT_HPP
#define UA_MAPF_EDGE_CONSTRAINT_HPP

#include "../common.hpp"

struct EdgeConstraint {
    int time;
    Location from;
    Location to;

    EdgeConstraint(int time, Location from, Location to);

    bool operator<(const EdgeConstraint& other) const;
    bool operator==(const EdgeConstraint& other) const;
    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c);
    inline friend size_t hash_value(EdgeConstraint const& c);
};



#endif //UA_MAPF_EDGE_CONSTRAINT_HPP
