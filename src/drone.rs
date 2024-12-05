use crate::helper;
use crossbeam_channel::{select, Receiver, Sender};
use rand::{thread_rng, Rng};
use std::collections::HashMap;
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodResponse, Nack, NackType, NodeType, Packet, PacketType};

pub struct LockheedRustin {
    id: NodeId,
    controller_send: Sender<DroneEvent>,
    controller_recv: Receiver<DroneCommand>,
    packet_recv: Receiver<Packet>,
    packet_send: HashMap<NodeId, Sender<Packet>>,
    pdr: f32,
    flood_archive: HashMap<NodeId, u64>,
}

impl Drone for LockheedRustin {
    fn new(
        id: NodeId,
        controller_send: Sender<DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32,
    ) -> Self {
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
                            self.handle_crash();
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
    /// Handle the given command.
    fn handle_command(&mut self, command: DroneCommand) {
        match command {
            DroneCommand::RemoveSender(node_id) => {
                self.packet_send.remove(&node_id);
            }
            DroneCommand::AddSender(node_id, sender) => {
                self.packet_send.insert(node_id, sender);
            }
            DroneCommand::SetPacketDropRate(pdr) => self.pdr = pdr,
            DroneCommand::Crash => unreachable!(),
        }
    }

    /// Handle the given packet.
    /// Check whether the routing is correct and, if it's correct, forward the packet appropriately.
    /// Otherwise, it sends a Nack packet to the packet sender instead.
    fn handle_packet(&mut self, packet: Packet) {
        match packet.pack_type {
            PacketType::MsgFragment(_) => match self.check_routing(&packet.routing_header) {
                Ok(_) => self.forward_fragment(packet),
                Err(nack_type) => self.handle_drop(packet, nack_type),
            },
            PacketType::FloodRequest(_) => self.handle_flood_request(packet),
            _ => match self.check_routing(&packet.routing_header) {
                Ok(_) => self.forward_packet(packet),
                Err(_) => self
                    .controller_send
                    .send(DroneEvent::ControllerShortcut(packet))
                    .unwrap(),
            },
        }
    }

    /// Handle the drone crashing.
    fn handle_crash(&mut self) {
        todo!()
    }

    /// Check if there has been an error in routing.
    /// Check if the packet has been delivered to the right destination (itself) and if the next hop is valid.
    fn check_routing(&self, header: &SourceRoutingHeader) -> Result<NodeId, NackType> {
        match helper::get_hop(&header) {
            Some(hop_index) if hop_index == self.id => match helper::get_next_hop(&header) {
                Some(next_hop) => {
                    if !self.packet_send.contains_key(&next_hop) {
                        Err(NackType::ErrorInRouting(next_hop))
                    } else {
                        Ok(next_hop)
                    }
                }
                None => Err(NackType::DestinationIsDrone),
            },
            _ => Err(NackType::UnexpectedRecipient(self.id)),
        }
    }

    /// Forward the fragment.
    fn forward_fragment(&self, packet: Packet) {
        if thread_rng().gen_range(0.0..=1.0) <= self.pdr {
            self.handle_drop(packet, NackType::Dropped);
        } else {
            self.forward_packet(packet);
        }
    }

    /// Forward all packets.
    fn forward_packet(&self, mut packet: Packet) {
        let next_hop = helper::get_next_hop(&packet.routing_header).unwrap();
        packet.routing_header.hop_index += 1;
        // todo: add check if next_hop is in neighbor
        match self.packet_send[&next_hop].send(packet.clone()) {
            Ok(_) => {
                self.controller_send
                    .send(DroneEvent::PacketSent(packet))
                    .unwrap();
            }
            Err(_) => {
                // this means that the drone has crashed
                packet.routing_header.hop_index -= 1;
                self.handle_drop(packet, NackType::ErrorInRouting(next_hop));
            }
        }
    }

    /// Create a new nack packet using the given Nack and the received packet to get the information about the session_id and the routing.
    /// If the packet cannot be dropped it's sent to the controller.
    fn handle_drop(&self, packet: Packet, nack_type: NackType) {
        match packet.pack_type {
            PacketType::MsgFragment(ref fragment) => {
                // create Nack path
                if packet.routing_header.hops.is_empty() {
                    return;
                }
                let mut hops = packet
                    .routing_header
                    .hops
                    .iter()
                    .cloned()
                    .take(packet.routing_header.hop_index + 1)
                    .rev()
                    .collect::<Vec<_>>();
                // force first id to be this drone id to fix unexpected recepient errors
                hops[0] = self.id;

                self.forward_packet(Packet {
                    pack_type: PacketType::Nack(Nack {
                        fragment_index: fragment.fragment_index,
                        nack_type,
                    }),
                    routing_header: SourceRoutingHeader { hop_index: 0, hops },
                    session_id: packet.session_id,
                });

                if let NackType::Dropped = nack_type {
                    self.controller_send
                        .send(DroneEvent::PacketDropped(packet))
                        .unwrap();
                }
            }
            _ => self
                .controller_send
                .send(DroneEvent::ControllerShortcut(packet))
                .unwrap(),
        }
    }

    /// Handle the given flood_request.
    fn handle_flood_request(&mut self, mut packet: Packet) {
        let PacketType::FloodRequest(mut flood_request) = packet.pack_type else {
            return;
        };
        let sender_id = flood_request.path_trace.last().unwrap().0;
        flood_request.path_trace.push((self.id, NodeType::Drone));
        packet.pack_type = PacketType::FloodRequest(flood_request.clone());

        match self.flood_archive.get(&flood_request.initiator_id) {
            Some(&flood_id) if flood_id == flood_request.flood_id => {
                self.flood_archive
                    .insert(flood_request.initiator_id.clone(), flood_request.flood_id);

                let flood_response_packet = Packet {
                    pack_type: PacketType::FloodResponse(FloodResponse {
                        flood_id,
                        path_trace: flood_request.path_trace.clone(),
                    }),
                    routing_header: SourceRoutingHeader {
                        hop_index: 1,
                        hops: flood_request
                            .path_trace
                            .iter()
                            .map(|(node_id, _)| *node_id)
                            .collect(),
                    },
                    session_id: packet.session_id,
                };

                self.forward_packet(flood_response_packet)
            }
            _ => {
                self.flood_archive
                    .insert(flood_request.initiator_id.clone(), flood_request.flood_id);

                for (node_id, sender) in self.packet_send.clone() {
                    if node_id != sender_id {
                        if let Ok(_) = sender.send(packet.clone()) {
                            self.controller_send
                                .send(DroneEvent::PacketSent(packet.clone()))
                                .unwrap();
                        }
                    }
                }
            }
        }
    }
}
