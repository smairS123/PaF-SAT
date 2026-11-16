#include "earliest_fail_minrc_direct_flow_node.hpp"

EarliestFailMinRCDirectFlowNode::EarliestFailMinRCDirectFlowNode(Type type, int time, Location location) :
        type(type), time(time), location(location) {}

EarliestFailMinRCDirectFlowNode::EarliestFailMinRCDirectFlowNode(Type type, int time) :
        EarliestFailMinRCDirectFlowNode(type, time, Location(-1, -1)) {}

bool EarliestFailMinRCDirectFlowNode::operator==(const EarliestFailMinRCDirectFlowNode &other) const {
    return type == other.type && time == other.time && location == other.location;
}

size_t hash_value(const EarliestFailMinRCDirectFlowNode &node) {
    size_t seed = 0;
    boost::hash_combine(seed, node.location);
    boost::hash_combine(seed, node.type);
    boost::hash_combine(seed, node.time);
    return seed;
}

std::ostream &operator<<(std::ostream &os, const EarliestFailMinRCDirectFlowNode &node) {
    switch (node.type) {
        case EarliestFailMinRCDirectFlowNode::OUT:
            return os << "(" << node.location << ", OUT, " << node.time << ")";
        case EarliestFailMinRCDirectFlowNode::IN:
            return os << "(" << node.location << ", IN, " << node.time << ")";
        case EarliestFailMinRCDirectFlowNode::SOURCE:
            return os << "(SOURCE)";
        case EarliestFailMinRCDirectFlowNode::SINK:
            return os << "(SINK)";
    }
    assert(false);
    return os;
}
