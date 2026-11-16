#ifndef UA_MAPF_EARLIEST_FAIL_MINRC_DIRECT_FLOW_NODE_HPP
#define UA_MAPF_EARLIEST_FAIL_MINRC_DIRECT_FLOW_NODE_HPP

#include "../common.hpp"

struct EarliestFailMinRCDirectFlowNode {
    enum Type {
        OUT = 0,
        IN = 1,
        SOURCE = 2,
        SINK = 3,
    };

    Type type;
    int time;
    Location location;

    EarliestFailMinRCDirectFlowNode(Type type, int time, Location location);
    EarliestFailMinRCDirectFlowNode(Type type, int time);

    bool operator==(const EarliestFailMinRCDirectFlowNode &other) const;
    friend size_t hash_value(const EarliestFailMinRCDirectFlowNode &node);
    friend std::ostream &operator<<(std::ostream &os, const EarliestFailMinRCDirectFlowNode &node);
};

#endif // UA_MAPF_EARLIEST_FAIL_MINRC_DIRECT_FLOW_NODE_HPP
