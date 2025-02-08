use std::{collections::HashMap, thread};

use client::Client;
use crossbeam_channel::{Receiver, Sender};
use fixedbitset::FixedBitSet;
use rust_roveri_api::{
    ClientChannels, ClientCommand, ClientEvent, ClientGuiMessage, ClientType, Command, Distros,
    DroneChannels, DroneImpl, GuiClientMessage, InitData, NodeType, ServerChannels, ServerCommand,
    ServerEvent, ServerType, MAX_CLIENT_TYPES, MAX_IMPL, MAX_NODES, MAX_SERVER_TYPES,
};
use server::Server;
use simulation_controller::factory::function::factory_drone;
use wg_2024::{
    config::Config,
    controller::{DroneCommand, DroneEvent},
    network::NodeId,
    packet::Packet,
};

/// Structure that encapsulates all data produced by the network initializer.
///
/// This data includes the initial network topology (as an `InitData` instance), the various
/// channels used to communicate with drones, clients, and servers, a list of GUI channels
/// for client nodes, and distribution data (via `Distros`) that specifies how many nodes of
/// each type are present.
#[derive(Clone, Debug)]
pub struct NetworkInitData {
    /// The initial network topology (nodes and their sender map).
    pub init_data: InitData,
    /// Channels for drone event communication.
    pub drone_channels: DroneChannels,
    /// Channels for client event communication.
    pub client_channels: ClientChannels,
    /// Channels for server event communication.
    pub server_channels: ServerChannels,
    /// For each client node, a tuple containing:
    /// `(NodeId, ClientType, Sender<GuiClientMessage>, Receiver<ClientGuiMessage>)`.
    /// These channels are used to spawn the corresponding client GUI (Browser or Chat).
    pub list_gui_channels: Vec<(
        NodeId,
        ClientType,
        Sender<GuiClientMessage>,
        Receiver<ClientGuiMessage>,
    )>,
    /// Distribution data for drones, clients, and servers.
    pub distros: Distros,
}

impl NetworkInitData {
    /// Returns a new instance of `NetworkInitData` from the given components.   
    ///
    /// This is the default, and only, constructor.
    ///
    /// # Parameters
    /// - `init_data`: The initial network topology data.
    /// - `drone_channels`: Channels used for drone communication.
    /// - `client_channels`: Channels used for client communication.
    /// - `server_channels`: Channels used for server communication.
    /// - `list_gui_channels`: A list of tuples for each client containing its ID, type, and GUI messaging channels.
    /// - `distros`: Distribution data for node types.
    pub fn new(
        init_data: InitData,
        drone_channels: DroneChannels,
        client_channels: ClientChannels,
        server_channels: ServerChannels,
        list_gui_channels: Vec<(
            NodeId,
            ClientType,
            Sender<GuiClientMessage>,
            Receiver<ClientGuiMessage>,
        )>,
        distros: Distros,
    ) -> Self {
        Self {
            init_data,
            drone_channels,
            client_channels,
            server_channels,
            list_gui_channels,
            distros,
        }
    }
}

/// Initializes the network by spawning all node threads and constructing the data structures
/// required both by the simulation controller and the GUI.   
///
/// Returns an istance of [`NetworkInitData`].
///
/// # Parameters
/// - `config`: A reference to the network configuration (parsed from the user’s configuration file).
///
/// # Behaviour
/// This function performs the following steps:
/// 1. **Topology Construction:**  
///    It creates three arrays:
///    - An array of `(NodeType, FixedBitSet)` representing the initial topology (with each
///      node’s type and its neighbor set).
///    - An array of `Command` instances (one per node) to store node-specific command channels.
///    - An array mapping node IDs to optional packet send channels (`Sender<Packet>`).
///
/// 2. **Channel Creation:**  
///    It creates unbounded channels for drones, clients, and servers that the simulation controller
///    will use to receive events.
///
/// 3. **Distribution Data:**  
///    It builds distribution arrays for drones, clients, and servers based on the configuration.
///    
/// 4. **GUI Channel Storage:**  
///    It prepares a vector of GUI channel tuples for client nodes (each containing the node ID, client type,
///    and channels for GUI communication).
///
/// 5. **Node Thread Spawning:**  
///    For each node (drone, client, and server) defined in the configuration:
///    - It sets up per-node command and packet channels.
///    - It assigns the node type into the topology array.
///    - It spawns a new thread that instantiates the node and then calls its `run()` method.
///
/// 6. **Graph Edge Construction:**  
///    After spawning nodes, it updates the topology graph by inserting neighbor edges for each node.
///    It also sends initial commands to each node to add the appropriate neighbor links.
///
/// 7. **Final Assembly:**  
///    Finally, it constructs an `InitData` instance from the topology, command array, and packet send map,
///    and wraps it together with the channels and distribution data in a `NetworkInitData` instance, which
///    is then returned.
pub fn network_init(config: &Config) -> NetworkInitData {
    // Create network topology data for the simulation controller:
    let mut topology: [(NodeType, FixedBitSet); MAX_NODES] =
        std::array::from_fn(|_index| (NodeType::None, FixedBitSet::with_capacity(MAX_NODES)));
    let mut senders: [Command; MAX_NODES] = std::array::from_fn(|_index| Command::None);
    let mut packet_send_map: [Option<Sender<Packet>>; MAX_NODES] =
        std::array::from_fn(|_index| None);

    // Create channels for the simulation controller to handle node events:
    let (drone_sender, drone_receiver) = crossbeam_channel::unbounded::<DroneEvent>();
    let (client_sender, client_receiver) = crossbeam_channel::unbounded::<ClientEvent>();
    let (server_sender, server_receiver) = crossbeam_channel::unbounded::<ServerEvent>();

    // Create distributions for the simulation controller:
    let mut drones_distro: [usize; MAX_IMPL] = std::array::from_fn(|_index| 0);
    let mut clients_distro: [usize; MAX_CLIENT_TYPES] = std::array::from_fn(|_index| 0);
    let mut servers_distro: [usize; MAX_SERVER_TYPES] = std::array::from_fn(|_index| 0);

    // Create an array to store the GUI channels for client nodes.
    let mut list_gui_channels: Vec<(
        NodeId,
        ClientType,
        Sender<GuiClientMessage>,
        Receiver<ClientGuiMessage>,
    )> = Vec::with_capacity(config.client.len());

    let mut index_drone_impl = 0;
    let mut index_client_types = 0;
    let mut index_server_types = 0;

    // Spawn drone threads.
    for drone in config.drone.iter().cloned() {
        let (sx_command, rx_command) = crossbeam_channel::unbounded::<DroneCommand>();
        let (sx_packet, rx_packet) = crossbeam_channel::unbounded::<Packet>();

        senders[drone.id as usize] = Command::DroneCommand(sx_command);
        packet_send_map[drone.id as usize] = Some(sx_packet);
        let drone_impl = DroneImpl::from_code(index_drone_impl).unwrap();
        drones_distro[index_drone_impl] += 1;
        index_drone_impl = (index_drone_impl + 1) % MAX_IMPL;
        topology[drone.id as usize].0 = NodeType::Drone(drone.pdr, drone_impl);

        // Spawn drone thread.
        let sender = drone_sender.clone();
        thread::spawn(move || {
            let mut drone = factory_drone(
                drone_impl,
                drone.id,
                sender,
                rx_command,
                rx_packet,
                HashMap::new(),
                drone.pdr,
            );
            drone.run();
        });
    }

    // Spawn client threads.
    for client in config.client.iter().cloned() {
        let (sx_command, rx_command) = crossbeam_channel::unbounded::<ClientCommand>();
        let (sx_packet, rx_packet) = crossbeam_channel::unbounded::<Packet>();
        let (message_sender_tx, message_sender_rx) =
            crossbeam_channel::unbounded::<GuiClientMessage>();
        let (message_receiver_tx, message_receiver_rx) =
            crossbeam_channel::unbounded::<ClientGuiMessage>();

        senders[client.id as usize] = Command::ClientCommand(sx_command);
        packet_send_map[client.id as usize] = Some(sx_packet);
        let client_type = ClientType::from_code(index_client_types).unwrap();
        clients_distro[index_client_types] += 1;
        index_client_types = (index_client_types + 1) % MAX_CLIENT_TYPES;
        topology[client.id as usize].0 = NodeType::Client(client_type);
        list_gui_channels.push((
            client.id,
            client_type,
            message_sender_tx,
            message_receiver_rx,
        ));

        // Spawn client thread.
        let sender = client_sender.clone();
        thread::spawn(move || {
            let mut client = Client::new(
                client.id,
                rx_packet,
                rx_command,
                sender,
                message_sender_rx,
                message_receiver_tx,
            );
            client.run();
        });
    }

    // Spawn server threads.
    for server in config.server.iter().cloned() {
        let (sx_command, rx_command) = crossbeam_channel::unbounded::<ServerCommand>();
        let (sx_packet, rx_packet) = crossbeam_channel::unbounded::<Packet>();

        senders[server.id as usize] = Command::ServerCommand(sx_command);
        packet_send_map[server.id as usize] = Some(sx_packet);
        let server_type = ServerType::from_code(index_server_types).unwrap();
        servers_distro[index_server_types] += 1;
        index_server_types = (index_server_types + 1) % MAX_SERVER_TYPES;
        topology[server.id as usize].0 = NodeType::Server(server_type);

        // Spawn server thread.
        let sender = server_sender.clone();
        thread::spawn(move || {
            let mut server = Server::new(server.id, rx_command, rx_packet, sender, server_type);
            server.run();
        });
    }

    // Update topology graph for drones.
    for drone in config.drone.iter().cloned() {
        for neighbor in &drone.connected_node_ids {
            topology[drone.id as usize].1.insert(*neighbor as usize);
            if let Command::DroneCommand(sender) = &senders[drone.id as usize] {
                let _ = sender.send(DroneCommand::AddSender(
                    *neighbor,
                    packet_send_map[*neighbor as usize]
                        .as_ref()
                        .unwrap()
                        .clone(),
                ));
            }
        }
    }
    // Update topology graph for clients.
    for client in config.client.iter() {
        for neighbor in &client.connected_drone_ids {
            topology[client.id as usize].1.insert(*neighbor as usize);
            if let Command::ClientCommand(sender) = &senders[client.id as usize] {
                let _ = sender.send(ClientCommand::AddDrone(
                    *neighbor,
                    packet_send_map[*neighbor as usize]
                        .as_ref()
                        .unwrap()
                        .clone(),
                ));
            }
        }
    }
    // Update topology graph for servers.
    for server in config.server.iter() {
        for neighbor in &server.connected_drone_ids {
            topology[server.id as usize].1.insert(*neighbor as usize);
            if let Command::ServerCommand(sender) = &senders[server.id as usize] {
                let _ = sender.send(ServerCommand::AddDrone(
                    *neighbor,
                    packet_send_map[*neighbor as usize]
                        .as_ref()
                        .unwrap()
                        .clone(),
                ));
            }
        }
    }

    // Create the initial data structure for the simulation controller.
    let init_data = InitData::new(topology, senders, packet_send_map);
    // Create communication channel wrappers.
    let drone_channels = DroneChannels::new(drone_receiver, drone_sender);
    let client_channels = ClientChannels::new(client_receiver, client_sender);
    let server_channels = ServerChannels::new(server_receiver, server_sender);
    // Create distribution data.
    let distros = Distros::new(drones_distro, clients_distro, servers_distro);

    // Assemble and return the complete network initialization data.
    NetworkInitData::new(
        init_data,
        drone_channels,
        client_channels,
        server_channels,
        list_gui_channels,
        distros,
    )
}
