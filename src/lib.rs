use crossbeam_channel::{select, Receiver, Sender};
use rand::{thread_rng, Rng};
use std::collections::HashMap;
use wg_2024::controller::Command;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Nack, NackType, Packet, PacketType};

pub struct LockheedRustin {
    pub id: u8,
    pub sim_contr_send: Sender<Command>,
    pub sim_contr_recv: Receiver<Command>,
    pub packet_recv: Receiver<Packet>,
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    pub pdr: f32,
}

impl Drone for LockheedRustin {
    fn new(options: DroneOptions) -> Self {
        Self {
            id: options.id,
            sim_contr_send: options.sim_contr_send,
            sim_contr_recv: options.sim_contr_recv,
            packet_recv: options.packet_recv,
            packet_send: HashMap::new(),
            pdr: options.pdr,
        }
    }
    fn run(&mut self) {
        loop {
            select! {
                recv(self.packet_recv) -> packet => {
                    if let Ok(packet) = packet {
                        self.handle_packet(packet);
                    }
                },
                recv(self.sim_contr_recv) -> command => {
                    if let Ok(command) = command {
                        if let Command::Crash = command {
                            break;
                        }
                        self.handle_command(command);
                    }
                }
            }
        }
    }
}

impl LockheedRustin {
    fn check_routing(&self, h: &mut SourceRoutingHeader) -> Result<NodeId, Nack> {
        if h.hop_index >= h.hops.len() || h.hops[h.hop_index] != self.id {
            return Err(Nack {
                fragment_index: 0,
                nack_type: NackType::UnexpectedRecipient(self.id),
            });
        }
        if h.hop_index + 1 == h.hops.len() {
            return Err(Nack {
                fragment_index: 0,
                nack_type: NackType::DestinationIsDrone,
            });
        }
        h.hop_index += 1;
        let next_hop = h.hops[h.hop_index];
        if !self.packet_send.contains_key(&next_hop) {
            return Err(Nack {
                fragment_index: 0,
                nack_type: NackType::ErrorInRouting(next_hop),
            });
        }
        return Ok(next_hop);
    }

    fn handle_packet(&mut self, mut packet: Packet) {
        if let Err(nack) = self.check_routing(&mut packet.routing_header) {
            return self.send_nack(packet, nack);
        }
        match packet.pack_type {
            PacketType::Nack(_nack) => unreachable!(),
            PacketType::Ack(_ack) => unreachable!(),
            PacketType::MsgFragment(_) => self.forward_packet(packet),
            PacketType::FloodRequest(_flood_request) => todo!(),
            PacketType::FloodResponse(_flood_response) => todo!(),
        }
    }

    fn handle_command(&mut self, command: Command) {
        match command {
            Command::AddChannel(_node_id, _sender) => todo!(),
            Command::RemoveChannel(_node_id) => todo!(),
            Command::Crash => unreachable!(),
        }
    }

    fn forward_packet(&self, packet: Packet) {
        if thread_rng().gen_range(0.0..1.0) <= self.pdr {
            return self.send_nack(
                packet,
                Nack {
                    fragment_index: 0,
                    nack_type: NackType::Dropped,
                },
            );
        }
        let next_hop = packet.routing_header.hops[packet.routing_header.hop_index];
        // TODO: is it safe to unwrap here?
        self.packet_send[&next_hop].send(packet).unwrap()
    }

    fn send_nack(&self, _packet: Packet, _nack: Nack) {
        unimplemented!()
    }

    /*
    fn forward_nack(&self, _packet: Packet, _nack: Nack) {
        unimplemented!()
    }
    fn forward_ack(&self, _packet: Packet, _ack: Ack) {
        unimplemented!()
    }
    */
}
