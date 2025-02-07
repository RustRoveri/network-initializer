use std::env;

use crossbeam_channel::Sender;
use rust_roveri_api::{ClientEvent, ClientType, DroneImpl, ServerEvent, ServerType, MAX_IMPL, MAX_NODES, MAX_SERVER_TYPES};
use validate::network_validate;
use wg_2024::{controller::DroneEvent, packet::Packet};

pub mod validate;
pub mod init;

#[test]
fn test_0() -> Result<(), String> {
    match env::current_dir() {
        Ok(path) => println!("Current working directory: {}", path.display()),
        Err(e) => println!("Error getting current directory: {}", e),
    }
    
    let network = network_validate("src/config.toml")?;

    let (drone_sender, drone_receiver) = crossbeam_channel::unbounded::<DroneEvent>();
    let (client_sender, client_receiver) = crossbeam_channel::unbounded::<ClientEvent>();
    let (server_sender, server_receiver) = crossbeam_channel::unbounded::<ServerEvent>();

    Ok(())
}