use crate::helper;
use crossbeam_channel::{select, Receiver, Sender};
use std::collections::HashMap;
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::{Drone};
use wg_2024::network::NodeId;
use wg_2024::packet::{Nack, NackType, Packet, PacketType};

pub struct LockheedRustin {
    pub id: u8,
    pub controller_send: Sender<DroneEvent>,
    pub controller_recv: Receiver<DroneCommand>,
    pub packet_recv: Receiver<Packet>,
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    pub pdr: f32,
}

impl Drone for LockheedRustin {
    fn new(id: NodeId,
           controller_send: Sender<DroneEvent>,
           controller_recv: Receiver<DroneCommand>,
           packet_recv: Receiver<Packet>,
           packet_send: HashMap<NodeId, Sender<Packet>>,
           pdr: f32,) -> Self {
        Self {
            id,
            controller_recv,
            controller_send,
            packet_recv,
            packet_send,
            pdr
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
    fn check_routing(&self, packet: &Packet) -> Result<NodeId, Nack> {
        match helper::get_hop(&packet) {
            Some(hop_index) if hop_index == self.id => match helper::get_next_hop(&packet) {
                Some(next_hop) => {
                    if !self.packet_send.contains_key(&next_hop) {
                        Err(Nack {
                            fragment_index: helper::get_fragment_id(packet),
                            nack_type: NackType::ErrorInRouting(next_hop),
                        })
                    } else {
                        Ok(next_hop)
                    }
                }
                None => Err(Nack {
                    fragment_index: helper::get_fragment_id(packet),
                    nack_type: NackType::DestinationIsDrone,
                }),
            },
            _ => Err(Nack {
                fragment_index: helper::get_fragment_id(packet),
                nack_type: NackType::UnexpectedRecipient(self.id),
            }),
        }
    }

    fn handle_packet(&mut self, mut packet: Packet) {
        if let Err(nack) = self.check_routing(&packet) {
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
            DroneCommand::RemoveSender(_) => unimplemented!(),
        }
    }

    fn forward_packet(&self, packet: Packet) {
        if helper::should_drop(&packet, self.pdr) {
            let fragment_index = helper::get_fragment_id(&packet);
            return self.send_nack(
                packet,
                Nack {
                    fragment_index,
                    nack_type: NackType::Dropped,
                },
            );
        }
        let next_hop = helper::get_next_hop(&packet).unwrap();
        if let Err(packet) = self.packet_send[&next_hop].send(packet) {
            let fragment_index = helper::get_fragment_id(&packet.0);
            self.send_nack(
                packet.0,
                Nack {
                    fragment_index,
                    nack_type: NackType::ErrorInRouting(next_hop),
                },
            );
        }
    }

    fn send_nack(&self, _packet: Packet, _nack: Nack) {
        unimplemented!()
    }
}
