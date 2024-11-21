use crossbeam_channel::{select, unbounded, Receiver, Sender};
use std::collections::HashMap;
use std::{fs, thread};
use wg_2024::config::Config;
use wg_2024::controller::Command;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::NodeId;
use wg_2024::packet::{Packet, PacketType};

/// TODO: move this example in the wgl repo
pub struct MyDrone {
    pub id: u8,
    pub sim_contr_send: Sender<Command>,
    pub sim_contr_recv: Receiver<Command>,
    pub packet_recv: Receiver<Packet>,
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    pub pdr: f32,
}

impl Drone for MyDrone {
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
                            println!("drone {} crashed", self.id);
                            break;
                        }
                        self.handle_command(command);
                    }
                }
            }
        }
    }
}

impl MyDrone {
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

struct SimulationController {
    drones: HashMap<NodeId, Sender<Command>>,
}

impl SimulationController {
    fn run(&mut self) {
        for (_, sender) in self.drones.iter() {
            sender.send(Command::Crash).unwrap();
        }
    }
}

fn parse_config(file: &str) -> Config {
    let file_str = fs::read_to_string(file).unwrap();
    toml::from_str(&file_str).unwrap()
}

fn main() {
    let config = parse_config("examples/config.toml");

    let mut packet_channels = HashMap::new();
    let mut command_channels = HashMap::new();

    let mut handles = Vec::new();

    for drone in config.drone.iter() {
        packet_channels.insert(drone.id, unbounded());
        command_channels.insert(drone.id, unbounded());
    }
    for client in config.client.iter() {
        packet_channels.insert(client.id, unbounded());
    }
    for server in config.server.iter() {
        packet_channels.insert(server.id, unbounded());
    }

    for drone in config.drone.into_iter() {
        let sim_contr_send = command_channels[&drone.id].0.clone();
        let sim_contr_recv = command_channels[&drone.id].1.clone();
        let packet_recv = packet_channels[&drone.id].1.clone();
        let packet_send = drone
            .connected_drone_ids
            .into_iter()
            .map(|id| (id, packet_channels[&id].0.clone()))
            .collect();

        handles.push(thread::spawn(move || {
            let mut drone = MyDrone::new(DroneOptions {
                id: drone.id,
                sim_contr_recv,
                sim_contr_send,
                packet_recv,
                pdr: drone.pdr,
            });
            drone.packet_send = packet_send;

            drone.run();
        }));
    }

    let mut controller = SimulationController {
        drones: command_channels
            .into_iter()
            .map(|(id, (send, _))| (id, send))
            .collect(),
    };
    controller.run();

    while let Some(handle) = handles.pop() {
        handle.join().unwrap();
    }
}
