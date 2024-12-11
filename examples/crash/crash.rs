use crossbeam_channel::unbounded;
use drone::LockheedRustin;
use std::{collections::HashMap, thread, time::Duration};
use wg_2024::{controller::DroneCommand, drone::Drone};

fn main() {
    let (controller_send, _) = unbounded();
    let (command_send, controller_recv) = unbounded();
    let (packet_send, packet_recv) = unbounded();
    let handle = thread::spawn(|| {
        let mut drone = LockheedRustin::new(
            1,
            controller_send,
            controller_recv,
            packet_recv,
            HashMap::new(),
            0.0,
        );
        drone.run();
        thread::sleep(Duration::from_secs(8));
    });
    command_send.send(DroneCommand::Crash).unwrap();
    drop(packet_send);
    handle.join().unwrap();
}
