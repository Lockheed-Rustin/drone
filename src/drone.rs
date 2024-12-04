use crate::helper;
use crossbeam_channel::{select, Receiver, Sender};
use std::collections::HashMap;
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::{Drone};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodRequest, FloodResponse, Nack, NackType, NodeType, Packet, PacketType};

pub type FloodArchive = HashMap<NodeId, u64>;

pub struct LockheedRustin {
    pub id: NodeId,
    pub controller_send: Sender<DroneEvent>,
    pub controller_recv: Receiver<DroneCommand>,
    pub packet_recv: Receiver<Packet>,
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    pub pdr: f32,
    pub flood_archive: FloodArchive,
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
            pdr,
            flood_archive: HashMap::new(),
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

    ///Handle the given command appropriately
    ///
    /// Parameters:
    /// command:DroneCommand -> command to handle
    fn handle_command(&mut self, command: DroneCommand) {
        match command {
            DroneCommand::AddSender(node_id, sender) => {
                self.packet_send.insert(node_id, sender);
            }
            DroneCommand::SetPacketDropRate(pdr) => self.pdr = pdr,
            DroneCommand::Crash => unreachable!(),
            DroneCommand::RemoveSender(node_id) => {
                self.packet_send.remove(&node_id);
            },
        }
    }


    /// Handle the given packet
    /// Check whether the routing is correct and, if it's correct, forward the packet appropriately
    /// Otherwise, it sends a Nack packet to the packet sender instead
    /// Parameters:
    /// packet:Packet -> packet to handle
    fn handle_packet(&mut self, mut packet: Packet) {
        match packet.pack_type {
            PacketType::FloodRequest(ref mut flood_request) => self.handle_flood_request(packet, flood_request),
            _ => {
                if let Err(nack) = self.check_routing(&packet) {
                    return match packet.pack_type {
                        PacketType::Nack(_) => {}
                        _ => self.send_nack(packet, nack),
                    };
                }
                if let PacketType::MsgFragment(_) = packet.pack_type {
                    self.forward_packet(packet);
                }
                else {
                    self.forward_ctrl_packet(packet)
                }
            }
        }
    }

    /// Check if there has been an error in routing.
    /// Check if the packet has been delivered to the right destination (itself) and if the next hop is valid
    ///
    /// Parameters:
    /// packet:Packet -> packet to forward
    ///
    /// Returns:
    /// Result<NodeId, Nack> -> the NodeId if the packet as well delivered and the next hop is valid
    ///                     -> an appropriate Nack for the relative problem that would have occurred during the routing otherwise
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

    /// Forward the given packet via the network
    /// Parameters:
    /// packet:Packet -> packet to forward
    fn forward_packet(&self, mut packet: Packet) {
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
        packet.routing_header.hop_index += 1;
        match self.packet_send[&next_hop].send(packet.clone()) {
            Ok(_) => {
                self.controller_send.send(DroneEvent::PacketSent(packet)).unwrap();
            }
            Err(mut packet) => {
                let fragment_index = helper::get_fragment_id(&packet.0);
                packet.0.routing_header.hop_index -= 1;
                self.send_nack(
                    packet.0,
                    Nack {
                        fragment_index,
                        nack_type: NackType::ErrorInRouting(next_hop),
                    },
                );
            }
        }
    }

    /// Create a new nack packet using the given Nack and the received packet to get the information about the session_id and the routing
    ///
    /// Parameters:
    /// packet:Packet -> the packet containing the session_id and the routing information
    /// nack:Nack -> the nack to forward
    fn send_nack(&self, packet: Packet, nack: Nack) {
        //create Nack packet
        let mut path: Vec<NodeId> = packet.routing_header.hops.iter().enumerate()
            .filter(|(i, _)| {*i < packet.routing_header.hop_index})
            .map(|(i, elem)| *elem)
            .collect();
        path.reverse();

        let nack_packet = Packet {
            pack_type: PacketType::Nack(nack),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: path,
            },
            session_id: packet.session_id,
        };

        //forward Nack Packet
        self.forward_ctrl_packet(nack_packet)
    }

    /// Forward all special packets that should not be dropped and that need to be sent to the simulation controller to avoid a crashed drone in their path.
    ///
    /// Parameters:
    /// packet:Packet -> the packet to forward
    fn forward_ctrl_packet(&self, mut packet: Packet) {
        let next_hop = helper::get_next_hop(&packet).unwrap();
        packet.routing_header.hop_index += 1;
        match self.packet_send[&next_hop].send(packet.clone()) {
            Ok(_) => {
                self.controller_send.send(DroneEvent::PacketSent(packet)).unwrap();
            }
            Err(_) => {
                self.controller_send.send(DroneEvent::ControllerShortcut(packet)).unwrap();
            }
        }
    }


    /// Handle the given flood_request
    ///
    /// Parameters:
    /// packet:Packet -> the packet containing the session_id and the routing information
    /// flood_request:FloodRequest -> the flood request to forward
    fn handle_flood_request(&mut self,  mut packet: Packet, flood_request: &mut FloodRequest) {
        let sender_id = flood_request.path_trace.last().unwrap().0;
        flood_request.path_trace.push((self.id, NodeType::Drone));

        match self.flood_archive.get(&flood_request.initiator_id) {
            Some(&flood_id) if flood_id == flood_request.flood_id => {
                self.flood_archive.insert(flood_request.initiator_id.clone(), flood_request.flood_id);

                let flood_response_packet = Packet {
                    pack_type: PacketType::FloodResponse(FloodResponse {
                        flood_id,
                        path_trace: flood_request.path_trace.clone(),
                    }),
                    routing_header: SourceRoutingHeader {
                        hop_index: 1,
                        hops: flood_request.path_trace.iter()
                            .map(|(nodeId, _)| {*nodeId})
                            .collect(),
                    },
                    session_id: packet.session_id,
                };

                self.forward_ctrl_packet(flood_response_packet)
            },
            _ => {
                self.flood_archive.insert(flood_request.initiator_id.clone(), flood_request.flood_id);

                for (node_id, sender) in self.packet_send {
                    if node_id != sender_id {
                        if let Ok(_) = sender.send(packet.clone()){
                            self.controller_send.send(DroneEvent::PacketSent(packet.clone())).unwrap();
                        }
                    }
                }
            }
        }
    }

}
