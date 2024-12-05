use wg_2024::network::{NodeId, SourceRoutingHeader};

/// Get the NodeId of the current hop
pub fn get_hop(header: &SourceRoutingHeader) -> Option<NodeId> {
    header.hops.get(header.hop_index).cloned()
}

/// Get the NodeId of the next hop
pub fn get_next_hop(header: &SourceRoutingHeader) -> Option<NodeId> {
    header.hops.get(header.hop_index + 1).cloned()
}
