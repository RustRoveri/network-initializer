use crossbeam_channel::{unbounded, Receiver, Sender};
use rust_roveri::RustRoveri;
use std::thread::{self, JoinHandle};
use std::collections::HashMap;
use wg_2024::config::Config;
use wg_2024::controller::DroneCommand;
use wg_2024::controller::DroneEvent;
use wg_2024::drone::Drone;
use wg_2024::network::NodeId;
use wg_2024::packet::Packet;

pub mod utils_;

struct SimulationController {
    events: Receiver<DroneEvent>,
    drones: HashMap<NodeId, Sender<DroneCommand>>,
    handles: HashMap<NodeId, JoinHandle<()>>,
}

fn spawn_nodes(config: Config) -> Result<SimulationController, String> {
    let (controller_send_tx, controller_send_rx) = unbounded::<DroneEvent>();
    let mut handles: HashMap<NodeId, JoinHandle<()>> = HashMap::new();
    let mut drones: HashMap<NodeId, Sender<DroneCommand>> = HashMap::new();

    let mut packet_channels = HashMap::new();
    for drone in &config.drone {
        packet_channels.insert(drone.id, unbounded::<Packet>());
    }
    for client in &config.client {
        packet_channels.insert(client.id, unbounded::<Packet>());
    }
    for server in &config.server {
        packet_channels.insert(server.id, unbounded::<Packet>());
    }

    for drone in config.drone {
        let packet_recv = match packet_channels.get(&drone.id) {
            Some((_, rx)) => rx.clone(),
            _ => return Err(String::from("Unexpected")),
        };

        let mut packet_send = HashMap::new();
        for adjacent in drone.connected_node_ids.iter() {
            if let Some((tx, _)) = packet_channels.get(adjacent) {
                packet_send.insert(*adjacent, tx.clone());
            } else {
                return Err(format!(
                    "Drone {} is trying to reference unexistent node {}",
                    drone.id, adjacent
                ));
            }
        }

        // Channel from controller to drone
        let (controller_recv_tx, controller_recv_rx) = unbounded::<DroneCommand>();
        drones.insert(drone.id, controller_recv_tx);

        // Spawn the drone
        let mut rust_roveri = RustRoveri::new(
            drone.id,
            controller_send_tx.clone(),
            controller_recv_rx,
            packet_recv,
            packet_send,
            drone.pdr,
        );
        let handle = thread::spawn(move || rust_roveri.run());
        handles.insert(drone.id, handle);
    }

    let controller = SimulationController {
        events: controller_send_rx,
        drones,
        handles,
    };

    Ok(controller)
}

// fn main() {
//     let config_data = fs::read_to_string("./config.toml").expect("Unable to read config file");
//     let config: Config = toml::from_str(config_data.as_str()).expect("Unable to parse TOML file");

//     println!("{:?}", config);

//     let controller = spawn_nodes(config).unwrap();

//     for (_, sender) in controller.drones {
//         let _ = sender.send(DroneCommand::Crash);
//     }

//     for handle in controller.handles {
//         let _ = handle.1.join();
//     }
// }

pub fn main() {}
