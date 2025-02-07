use std::{collections::HashMap, thread};

use client::Client;
use crossbeam_channel::{Receiver, Sender};
use fixedbitset::FixedBitSet;
use rust_roveri_api::{ClientChannels, ClientCommand, ClientEvent, ClientGuiMessage, ClientType, Command, Distros, DroneChannels, DroneImpl, GUIChannels, GUIRequest, GUIResponse, GuiClientMessage, InitData, NodeType, ServerChannels, ServerEvent, ServerType, MAX_CLIENT_TYPES, MAX_IMPL, MAX_NODES, MAX_SERVER_TYPES};
use simulation_controller::factory::function::factory_drone;
use wg_2024::{config::Config, controller::{DroneCommand, DroneEvent}, packet::{self, Packet}};

pub struct NetworkInitData {
    init_data: InitData,
    drone_channels: DroneChannels,
    client_channels: ClientChannels,
    server_channels: ServerChannels,
    list_gui_channels: Vec<(NodeId, ClientType, Sender<GuiClientMessage>, Receiver<ClientGuiMessage>)>,
    distros: Distros
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



pub fn network_init(config: &Config) -> Result<NetworkInitData, String> {
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
    let mut drones_distro: [(usize, DroneImpl); MAX_IMPL] =
        std::array::from_fn(|index| (0, DroneImpl::from_code(index).unwrap()));
    let mut clients_distro: [(usize, ClientType); 2] =
        [(0, ClientType::Browser), (1, ClientType::Chat)];
    let mut servers_distro: [(usize, ServerType); MAX_SERVER_TYPES] = [
        (0, ServerType::Chat),
        (0, ServerType::ContentMedia),
        (0, ServerType::ContentText),
    ];

    let mut index_drones_impl = 0;
    let mut index_clients_types = 0;
    let mut index_server_types = 0;

    for drone in config.drone.iter() {
        let (sx_command, rx_command) = crossbeam_channel::unbounded::<DroneCommand>();
        let (sx_packet, rx_packet) = crossbeam_channel::unbounded::<Packet>();

        topology[drone.id as usize].0 = NodeType::Drone;
        senders[drone.id as usize] = Command::DroneCommand(sx_command);
        packet_send_map[drone.id as usize] = Some(sx_packet);


        thread::spawn(move || {
            let mut drone = factory_drone(DroneImpl::from_code(index_drones_impl), drone.id, drone_sender.clone(), rx_command, rx_packet, HashMap::new(), drone.pdr);
        });
    }

    for client in config.client.iter() {
        let (sx_command, rx_command) = crossbeam_channel::unbounded::<ClientCommand>();
        let (sx_packet, rx_packet) = crossbeam_channel::unbounded::<Packet>();
        let (message_sender_tx, message_sender_rx) = crossbeam_channel::unbounded::<GuiClientMessage>();
        let (message_receiver_tx, message_receiver_rx) = crossbeam_channel::unbounded::<ClientGuiMessage>();

        if index_clients_types == 0 {
            // Browser 
            thread::spawn(move || {
                let mut client = Client::new(client.id, )
            })
        } else {
            // Chat
        }
        clients_distro[index_clients_types] 
        index_clients_types += 1;
        index_clients_types %= MAX_CLIENT_TYPES;
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
    let res = NetworkInitData::new(init_data, drone_channels, client_channels, server_channels, distros);

    Ok(res)
}