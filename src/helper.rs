use wg_2024::network::{NodeId, SourceRoutingHeader};

/// Get the NodeId of the current hop
pub fn get_hop(header: &SourceRoutingHeader) -> Option<NodeId> {
    header.hops.get(header.hop_index).cloned()
}

/// Get the NodeId of the next hop
pub fn get_next_hop(header: &SourceRoutingHeader) -> Option<NodeId> {
    header.hops.get(header.hop_index + 1).cloned()
}

/// Get the simplified version of the route of a FloodResponse
pub fn simplify_hops(hops: &mut Vec<u8>) {
    let src_id = hops[0];
    let cycle_index = hops.iter().rposition(|hop| *hop == src_id).unwrap();
    hops.drain(0..cycle_index);
}
