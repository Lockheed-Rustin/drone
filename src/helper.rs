use rand::{thread_rng, Rng};
use wg_2024::{
    network::NodeId,
    packet::{Packet, PacketType},
};

/*
Get the fragment:id of the given packet

Parameters:
packet:Packet -> package from which to take the fragment id

Returns:
u64 -> the fragment_id of the packet
*/
pub fn get_fragment_id(packet: &Packet) -> u64 {
    match &packet.pack_type {
        PacketType::MsgFragment(f) => f.fragment_index,
        _ => 0,
    }
}

/*
Get the NodeId of the current hop

Parameters:
packet:Packet -> package from which to take the current hop node

Returns:
Option<NodeId> -> returns the NodeId of the current hop if exists, None otherwise
*/
pub fn get_hop(packet: &Packet) -> Option<NodeId> {
    packet
        .routing_header
        .hops
        .get(packet.routing_header.hop_index)
        .cloned()
}

/*
Get the NodeId of the next hop

Parameters:
packet:Packet -> package from which to take the current hop node

Returns:
Option<NodeId> -> returns the NodeId of the current hop if exists, None otherwise
*/
pub fn get_next_hop(packet: &Packet) -> Option<NodeId> {
    packet
        .routing_header
        .hops
        .get(packet.routing_header.hop_index + 1)
        .cloned()
}

/*
Determines if the packet that is a MsgFragment should be dropped based on the pdr

Parameters:
packet:Packet -> the packet that could be dropped
pdr: f32 -> the packet drop probability

Returns:
bool -> return if the packet should be dropped
*/
pub fn should_drop(packet: &Packet, pdr: f32) -> bool {
    match &packet.pack_type {
        PacketType::MsgFragment(_) => thread_rng().gen_range(0.0..1.0) <= pdr,
        _ => false,
    }
}
