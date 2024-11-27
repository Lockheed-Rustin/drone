use rand::{thread_rng, Rng};
use wg_2024::{
    network::{NodeId, SourceRoutingHeader},
    packet::{Packet, PacketType},
};

pub fn get_fragment_id(packet: &Packet) -> Option<u64> {
    match &packet.pack_type {
        PacketType::MsgFragment(f) => Some(f.fragment_index),
        _ => None,
    }
}

pub fn get_hop(h: &SourceRoutingHeader) -> Option<NodeId> {
    h.hops.get(h.hop_index).cloned()
}

pub fn get_next_hop(h: &SourceRoutingHeader) -> Option<NodeId> {
    h.hops.get(h.hop_index + 1).cloned()
}

pub fn should_drop(packet: &Packet, pdr: f32) -> bool {
    if let PacketType::MsgFragment(_) = packet.pack_type {
        thread_rng().gen_range(0.0..1.0) <= pdr
    } else {
        false
    }
}
