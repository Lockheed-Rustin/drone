use crossbeam_channel::{select, Receiver, Sender};
use std::collections::HashMap;
use wg_2024::controller::Command;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::NodeId;
use wg_2024::packet::{Packet, PacketType};

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
    fn handle_packet(&mut self, packet: Packet) {
        match packet.pack_type {
            PacketType::Nack(_nack) => todo!(),
            PacketType::Ack(_ack) => todo!(),
            PacketType::MsgFragment(_fragment) => todo!(),
            PacketType::Flood(_query) => todo!(),
            PacketType::FloodResult(_query_result) => todo!(),
        }
    }
    fn handle_command(&mut self, command: Command) {
        match command {
            Command::AddChannel(_node_id, _sender) => todo!(),
            Command::RemoveChannel(_node_id) => todo!(),
            Command::Crash => unreachable!(),
        }
    }
}
