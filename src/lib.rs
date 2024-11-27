use crossbeam_channel::{select, Receiver, Sender};
use std::collections::HashMap;
use wg_2024::controller::{DroneCommand, NodeEvent};
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Nack, Packet, PacketType};
mod helper;

pub struct LockheedRustin {
    pub id: u8,
    pub controller_send: Sender<NodeEvent>,
    pub controller_recv: Receiver<DroneCommand>,
    pub packet_recv: Receiver<Packet>,
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    pub pdr: f32,
}

impl Drone for LockheedRustin {
    fn new(options: DroneOptions) -> Self {
        Self {
            id: options.id,
            controller_send: options.controller_send,
            controller_recv: options.controller_recv,
            packet_recv: options.packet_recv,
            packet_send: options.packet_send,
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
                recv(self.controller_recv) -> command => {
                    if let Ok(command) = command {
                        if let DroneCommand::Crash = command {
                            return;
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
        match helper::get_hop(h) {
            Some(hop_index) if hop_index == self.id => match helper::get_next_hop(h) {
                Some(next_hop) => {
                    if !self.packet_send.contains_key(&next_hop) {
                        Err(Nack::ErrorInRouting(next_hop))
                    } else {
                        Ok(next_hop)
                    }
                }
                None => Err(Nack::DestinationIsDrone),
            },
            _ => Err(Nack::UnexpectedRecipient(self.id)),
        }
    }

    fn handle_packet(&mut self, mut packet: Packet) {
        if let Err(nack) = self.check_routing(&mut packet.routing_header) {
            return match packet.pack_type {
                PacketType::Nack(_) => {}
                _ => self.send_nack(packet, nack),
            };
        }
        match packet.pack_type {
            PacketType::Nack(_nack) => todo!(),
            PacketType::Ack(_ack) => todo!(),
            PacketType::MsgFragment(_) => {
                packet.routing_header.hop_index += 1;
                self.forward_packet(packet);
            }
            PacketType::FloodRequest(_flood_request) => todo!(),
            PacketType::FloodResponse(_flood_response) => todo!(),
        }
    }

    fn handle_command(&mut self, command: DroneCommand) {
        match command {
            DroneCommand::AddSender(node_id, sender) => {
                self.packet_send.insert(node_id, sender);
            }
            DroneCommand::SetPacketDropRate(pdr) => self.pdr = pdr,
            DroneCommand::Crash => unreachable!(),
        }
    }

    fn forward_packet(&self, packet: Packet) {
        if helper::should_drop(&packet, self.pdr) {
            let fragment_index = helper::get_fragment_id(&packet).unwrap();
            return self.send_nack(packet, Nack::Dropped(fragment_index));
        }
        let next_hop = helper::get_next_hop(&packet.routing_header).unwrap();
        if let Err(packet) = self.packet_send[&next_hop].send(packet) {
            self.send_nack(packet.0, Nack::ErrorInRouting(next_hop));
        }
    }

    fn send_nack(&self, _packet: Packet, _nack: Nack) {
        unimplemented!()
    }
}
