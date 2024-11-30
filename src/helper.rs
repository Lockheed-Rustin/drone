use rand::{thread_rng, Rng};
use wg_2024::{
    network::NodeId,
    packet::{Packet, PacketType},
};

pub fn get_fragment_id(packet: &Packet) -> u64 {
    match &packet.pack_type {
        PacketType::MsgFragment(f) => f.fragment_index,
        _ => 0,
    }
}

pub fn get_hop(packet: &Packet) -> Option<NodeId> {
    packet
        .routing_header
        .hops
        .get(packet.routing_header.hop_index)
        .cloned()
}

pub fn get_next_hop(packet: &Packet) -> Option<NodeId> {
    packet
        .routing_header
        .hops
        .get(packet.routing_header.hop_index + 1)
        .cloned()
}

pub fn should_drop(packet: &Packet, pdr: f32) -> bool {
    match &packet.pack_type {
        PacketType::MsgFragment(_) => thread_rng().gen_range(0.0..1.0) <= pdr,
        _ => false,
    }
}
