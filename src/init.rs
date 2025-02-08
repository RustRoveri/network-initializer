use std::{collections::HashMap, thread};

use client::Client;
use crossbeam_channel::{Receiver, Sender};
use fixedbitset::FixedBitSet;
use rust_roveri_api::{ClientChannels, ClientCommand, ClientEvent, ClientGuiMessage, ClientType, Command, Distros, DroneChannels, DroneImpl, GUIChannels, GUIRequest, GUIResponse, GuiClientMessage, InitData, NodeType, ServerChannels, ServerCommand, ServerEvent, ServerType, MAX_CLIENT_TYPES, MAX_IMPL, MAX_NODES, MAX_SERVER_TYPES};
use server::Server;
use simulation_controller::factory::function::factory_drone;
use wg_2024::{config::Config, controller::{DroneCommand, DroneEvent}, network::NodeId, packet::Packet};

pub struct NetworkInitData {
    pub init_data: InitData,
    pub drone_channels: DroneChannels,
    pub client_channels: ClientChannels,
    pub server_channels: ServerChannels,
    pub list_gui_channels: Vec<(NodeId, ClientType, Sender<GuiClientMessage>, Receiver<ClientGuiMessage>)>,
    pub distros: Distros
}

impl NetworkInitData {
    pub fn new(
        init_data: InitData,
        drone_channels: DroneChannels, 
        client_channels: ClientChannels, 
        server_channels: ServerChannels, 
        list_gui_channels: Vec<(NodeId, ClientType, Sender<GuiClientMessage>, Receiver<ClientGuiMessage>)>,
        distros: Distros
    ) -> Self {
        Self {
            init_data, 
            drone_channels, 
            client_channels, 
            server_channels,
            list_gui_channels,
            distros
        }
    }
}


pub fn network_init(config: &Config) -> NetworkInitData {
    // Create network topology data for the SC
    let mut topology: [(NodeType, FixedBitSet); MAX_NODES] = std::array::from_fn(|_index| {
        (NodeType::None, FixedBitSet::with_capacity(MAX_NODES))
    });
    let mut senders: [Command; MAX_NODES] = std::array::from_fn(|_index| {
        Command::None
    });
    let mut packet_send_map: [Option<Sender<Packet>>; MAX_NODES] = std::array::from_fn(|_index| {
        None
    });

    // Create channels for the SC to handle node events 
    let (drone_sender, drone_receiver) = crossbeam_channel::unbounded::<DroneEvent>();
    let (client_sender, client_receiver) = crossbeam_channel::unbounded::<ClientEvent>();
    let (server_sender, server_receiver) = crossbeam_channel::unbounded::<ServerEvent>();
    
    // Create distributions for the SC
    let mut drones_distro: [usize; MAX_IMPL] = std::array::from_fn(|_index| { 
        0
    });
    let mut clients_distro: [usize; MAX_CLIENT_TYPES] = std::array::from_fn(|_index| {
        0
    });
    let mut servers_distro: [usize; MAX_SERVER_TYPES] = std::array::from_fn(|_index| {
        0
    });

    // Create array to store the gui_channels for clients 
    let mut list_gui_channels:Vec<(NodeId, ClientType, Sender<GuiClientMessage>, Receiver<ClientGuiMessage>)> = Vec::with_capacity(config.client.len());

    let mut index_drone_impl = 0;
    let mut index_client_types = 0;
    let mut index_server_types = 0;

    // cloned() and the cloning of the sender are necessary due to the `move` keyword
    for drone in config.drone.iter().cloned() {
        let (sx_command, rx_command) = crossbeam_channel::unbounded::<DroneCommand>();
        let (sx_packet, rx_packet) = crossbeam_channel::unbounded::<Packet>();

        senders[drone.id as usize] = Command::DroneCommand(sx_command);
        packet_send_map[drone.id as usize] = Some(sx_packet);
        let drone_impl = DroneImpl::from_code(index_drone_impl).unwrap();
        drones_distro[index_drone_impl] += 1;
        index_drone_impl += 1;
        index_drone_impl %= MAX_IMPL;
        topology[drone.id as usize].0 = NodeType::Drone(drone_impl);

        // Spawn drone
        let sender = drone_sender.clone();
        thread::spawn(move || {
            let mut drone = factory_drone(drone_impl, drone.id, sender, rx_command, rx_packet, HashMap::new(), drone.pdr);
            drone.run();
        });
    }

    // cloned() and the cloning of the sender are necessary due to the `move` keyword
    for client in config.client.iter().cloned() {
        let (sx_command, rx_command) = crossbeam_channel::unbounded::<ClientCommand>();
        let (sx_packet, rx_packet) = crossbeam_channel::unbounded::<Packet>();
        let (message_sender_tx, message_sender_rx) = crossbeam_channel::unbounded::<GuiClientMessage>();
        let (message_receiver_tx, message_receiver_rx) = crossbeam_channel::unbounded::<ClientGuiMessage>();

        senders[client.id as usize] = Command::ClientCommand(sx_command);
        packet_send_map[client.id as usize] = Some(sx_packet);
        let client_type = ClientType::from_code(index_client_types).unwrap();
        clients_distro[index_client_types] += 1;
        index_client_types += 1;
        index_client_types %= MAX_CLIENT_TYPES;
        topology[client.id as usize].0 = NodeType::Client(client_type);
        list_gui_channels.push((client.id, client_type, message_sender_tx, message_receiver_rx));

        // Spawn client
        let sender = client_sender.clone();
        thread::spawn(move || {
            let mut client = Client::new(client.id, rx_packet, rx_command, sender, message_sender_rx, message_receiver_tx);
            client.run();
        });
    }

    // cloned() and the cloning of the sender are necessary due to the `move` keyword
    for server in config.server.iter().cloned() {
        let (sx_command, rx_command) = crossbeam_channel::unbounded::<ServerCommand>();
        let (sx_packet, rx_packet) = crossbeam_channel::unbounded::<Packet>();

        senders[server.id as usize] = Command::ServerCommand(sx_command);
        packet_send_map[server.id as usize] = Some(sx_packet);
        let server_type = ServerType::from_code(index_server_types).unwrap();
        servers_distro[index_server_types] += 1;
        index_server_types += 1;
        index_server_types %= MAX_SERVER_TYPES;
        topology[server.id as usize].0 = NodeType::Server(server_type);

        // Spawn server 
        let sender = server_sender.clone();
        thread::spawn(move || {
            let mut server = Server::new(server.id, rx_command, rx_packet, sender, server_type);
            server.run();
        });
    }

    for drone in config.drone.iter().cloned() {
        for neighbor in &drone.connected_node_ids {
            // Update topology
            topology[drone.id as usize].1.insert(*neighbor as usize);
            if let Command::DroneCommand(sender) = &senders[drone.id as usize] {
                let _ = sender.send(DroneCommand::AddSender(*neighbor, packet_send_map[*neighbor as usize].as_ref().unwrap().clone()));
            }
        }
    }
    for client in config.client.iter() {
        for neighbor in &client.connected_drone_ids {
            // Update topology
            topology[client.id as usize].1.insert(*neighbor as usize);
            if let Command::ClientCommand(sender) = &senders[client.id as usize] {
                let _ = sender.send(ClientCommand::AddDrone(*neighbor, packet_send_map[*neighbor as usize].as_ref().unwrap().clone()));
            }
        }
    }
    for server in config.server.iter() {
        for neighbor in &server.connected_drone_ids {
            // Update topology
            topology[server.id as usize].1.insert(*neighbor as usize);
            if let Command::ServerCommand(sender) = &senders[server.id as usize] {
                let _ = sender.send(ServerCommand::AddDrone(*neighbor, packet_send_map[*neighbor as usize].as_ref().unwrap().clone()));
            }
        }
    }

    // Create InitData 
    let init_data = InitData::new(topology, senders, packet_send_map);
    // Create DroneChannels
    let drone_channels = DroneChannels::new(drone_receiver, drone_sender);
    // Create ClientChannels
    let client_channels = ClientChannels::new(client_receiver, client_sender);
    // Create ServerChannels
    let server_channels = ServerChannels::new(server_receiver, server_sender);
    // Create Distros
    let distros = Distros::new(drones_distro, clients_distro, servers_distro);

    // Create NetworkInitData
    NetworkInitData::new(
        init_data, 
        drone_channels, 
        client_channels, 
        server_channels, 
        list_gui_channels, 
        distros
    )
}