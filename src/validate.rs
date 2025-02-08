use std::{collections::VecDeque, fs};

use fixedbitset::FixedBitSet;
use rust_roveri_api::MAX_NODES;
use wg_2024::config::{Client, Config, Drone, Server};

type Graph = [FixedBitSet; MAX_NODES];

/// Compute the initialization of the network.   
///
/// # Parameters
/// - `file_path` - Path of the configuration file.
pub fn network_validate(file_path: &str) -> Result<Config, String> {
    // Deserializing the config file
    let config_data = fs::read_to_string(file_path)
        .map_err(|_| "Unable to read configuration file".to_string())?;
    let config: Config =
        toml::from_str(&config_data).map_err(|e| format!("Failed to deserialize TOML: {}", e))?;

    // Validate the configuration
    validate_config(&config)?;

    Ok(config)
}

/// Checks if the config. file encodes a proper network topology.  
///
/// # Parameters
/// - `config` - The configuration of the network.
///
/// # Performance
/// - `O(n + m)`, where `n` and `m` are the number of nodes and edges in the network, respectively.
fn validate_config(config: &Config) -> Result<(), String> {
    let mut n_nodes = 0;
    // By default all setted to 0
    let mut node_ids = FixedBitSet::with_capacity(MAX_NODES);

    // Validate drones
    for drone in &config.drone {
        validate_drone(drone)?;
        if node_ids.contains(drone.id as usize) {
            return Err(format!("Duplicate node ID found: [{}]", drone.id).into());
        } else {
            node_ids.insert(drone.id as usize);
            n_nodes += 1;
        }
    }
    let drone_ids = node_ids.clone();
    let n_drones = n_nodes;

    // Validate clients
    for client in &config.client {
        validate_client(client)?;
        if node_ids.contains(client.id as usize) {
            return Err(format!("Duplicate node ID found: [{}]", client.id).into());
        } else {
            node_ids.insert(client.id as usize);
            n_nodes += 1;
        }
    }

    // Validate servers
    for server in &config.server {
        validate_server(server)?;
        if node_ids.contains(server.id as usize) {
            return Err(format!("Duplicate node ID found: [{}]", server.id).into());
        } else {
            node_ids.insert(server.id as usize);
            n_nodes += 1;
        }
    }

    assert_eq!(
        n_nodes,
        config.drone.len() + config.client.len() + config.server.len()
    );

    // Check for each client/server if its neighbors are all drones
    validate_all_neighbors_are_drones(config, &drone_ids)?;

    let mut graph: Graph = std::array::from_fn(|_| FixedBitSet::with_capacity(MAX_NODES));

    compute_init_graph(&mut graph, config);

    validate_bidirectional_graph(&graph, &node_ids)?;
    validate_connected_graph(&graph, &node_ids, n_nodes)?;
    validate_edges_clients_servers(&graph, &drone_ids, n_nodes, n_drones)?;

    Ok(())
}

/// Checks if the drone has a proper PDR and if its neighbors list does not contain duplicates.   
///
/// # Parameter
/// - `drone` - Drone to be checked.
///
/// # Performance
/// - `O(n)`, where `n` is the number of nodes in the network.
fn validate_drone(drone: &Drone) -> Result<(), String> {
    // Checks if the PDR is in bounds
    if drone.pdr < 0_f32 || drone.pdr > 1_f32 {
        return Err(format!("Invalid PDR for drone [{}]: {}", drone.id, drone.pdr).into());
    }

    // Checks if the drone itself is in the list of its neighbors and if there are not duplicates in the list
    let mut set = FixedBitSet::with_capacity(MAX_NODES);
    for connected_id in &drone.connected_node_ids {
        if *connected_id == drone.id {
            return Err(format!("Drone [{}] is connected to itself", drone.id).into());
        }
        if set.contains(*connected_id as usize) {
            return Err(format!(
                "Drone [{}] has [{}] as duplicate in its neighbors list",
                drone.id, *connected_id
            ));
        }
        set.insert(*connected_id as usize);
    }

    Ok(())
}

/// Checks if the client is connected to at least 1 node and at max 2 nodes, and that its neighbors list does not   
/// contain duplicates.  
///
/// # Parameters
/// - `client` - Client to be checked.
///
/// # Performance
/// - `O(n)`, where `n` is the number of nodes in the network.
fn validate_client(client: &Client) -> Result<(), String> {
    // Checks if the server is connected to 0 drones
    if client.connected_drone_ids.is_empty() {
        return Err(format!("Client [{}] is connected to 0 drones", client.id).into());
    }
    // Checks if the server is connected to n > 2 drones
    if client.connected_drone_ids.len() > 2 {
        return Err(format!("Client [{}] has more than 2 neighbors", client.id).into());
    }

    // Checks if the client itself is in the list of its neighbors and if there are not duplicates in the list
    let mut set = FixedBitSet::with_capacity(MAX_NODES);
    for connected_id in &client.connected_drone_ids {
        if *connected_id == client.id {
            return Err(format!("Client [{}] is connected to itself", client.id).into());
        }
        if set.contains(*connected_id as usize) {
            return Err(format!(
                "Client [{}] has a duplicate in its neighbors list: [{}]",
                client.id, *connected_id
            ));
        }
        set.insert(*connected_id as usize);
    }

    Ok(())
}

/// Checks if the server is connected to at least 2 nodes, and that its neighbors list does not   
/// contain duplicates.  
///
/// # Parameters
/// - `server` - Server to be checked.
///
/// # Performance
/// - `O(n)`, where `n` is the number of nodes in the network.
fn validate_server(server: &Server) -> Result<(), String> {
    // Checks if the server is connected to n > 2 drones
    if server.connected_drone_ids.len() < 2 {
        return Err(format!("Server [{}] has less than 2 neighbors", server.id));
    }

    // Checks if the server itself is in the list of its neighbors and if there are not duplicates in the list
    let mut set = FixedBitSet::with_capacity(MAX_NODES);
    for connected_id in &server.connected_drone_ids {
        if *connected_id == server.id {
            return Err(format!("Server [{}] is connected to itself", server.id).into());
        }
        if set.contains(*connected_id as usize) {
            return Err(format!(
                "Server [{}] has a duplicate in its neighbors list: [{}]",
                server.id, *connected_id
            ));
        }
        set.insert(*connected_id as usize);
    }

    Ok(())
}

/// For every server and client checks if their neighbors list contains only ids of drones.   
///
/// # Parameters
/// - `config` - The configuration of the topology.
/// - `drone_ids` - Set with all the drone ids.
///
/// # Performance
/// - `O(n + m)`, where `n` and `m` are the number of nodes and edges in the network, respectively.
fn validate_all_neighbors_are_drones(
    config: &Config,
    drone_ids: &FixedBitSet,
) -> Result<(), String> {
    // Client check
    for client in &config.client {
        for id in &client.connected_drone_ids {
            if !drone_ids.contains(*id as usize) {
                return Err(format!(
                    "Client [{}] is connected to [{}], which is not a drone",
                    client.id, *id
                )
                .into());
            }
        }
    }

    // Server check
    for server in &config.server {
        for id in &server.connected_drone_ids {
            if !drone_ids.contains(*id as usize) {
                return Err(format!(
                    "Server [{}] is connected to [{}], which is not a drone",
                    server.id, *id
                )
                .into());
            }
        }
    }

    Ok(())
}

/// Creates the graph of the network.   
///
/// # Parameters
/// - `graph` - Mutable reference to the graph.
/// - `config` - Configuration of the network.
///
/// # Performance
/// - `O(n + m)`, where `n` and `m` are the number of nodes and edges in the network, respectively.
fn compute_init_graph(graph: &mut Graph, config: &Config) {
    // Stores the drones and their connections
    for drone in &config.drone {
        for id in &drone.connected_node_ids {
            graph[drone.id as usize].insert(*id as usize);
        }
    }

    // Stores the clients and their connections
    for client in &config.client {
        for id in &client.connected_drone_ids {
            graph[client.id as usize].insert(*id as usize);
        }
    }

    // Stores the servers and their connections
    for server in &config.server {
        for id in &server.connected_drone_ids {
            graph[server.id as usize].insert(*id as usize);
        }
    }
}

/// Checks if the graph of the network is bidirectional.  
///
/// # Parameters
/// - `graph` - Graph representing the network.
/// - `node_ids` - Every node id in the network.
///
/// # Performance
/// - `O(n + m)`, where `n` and `m` are the number of nodes and edges in the network, respectively.
fn validate_bidirectional_graph(graph: &Graph, node_ids: &FixedBitSet) -> Result<(), String> {
    for node in node_ids.ones() {
        // We are considering the edge [node] -> [id]
        for id in graph[node].ones() {
            // Edge case: checks if the node id is in the topology, ie if the user has specified it as a node in the config file, O(1)
            if !node_ids.contains(id) {
                return Err(format!(
                    "Node [{}] has [{}] as neighbor, which does not exist in the topology.",
                    node, id
                )
                .into());
            }
            // Checks if the edge [id] -> [node], O(1)
            if !graph[id].contains(node) {
                return Err(format!(
                    "The topology does not represent a bidirectional graph:\nnode [{}] is reachable from [{}], but not viceversa.",
                    id, node
                )
                .into());
            }
        }
    }

    Ok(())
}

/// Checks if the network graph is connected, ie it has only one connected component.
///
/// # Parameters
/// - `graph` - Graph of the network.
/// - `node_ids` - Set of all the node ids in the network.
/// - `n_nodes` - Number of nodes in the network.
///
/// # Performance
/// - `O(n + m)`, where `n` and `m` are the number of nodes and edges in the network, respectively.
fn validate_connected_graph(
    graph: &Graph,
    node_ids: &FixedBitSet,
    n_nodes: usize,
) -> Result<(), String> {
    if n_nodes == 0 {
        return Ok(());
    }

    let mut visited = FixedBitSet::with_capacity(MAX_NODES);
    let mut n_visited = 0;
    let node = node_ids.ones().next().unwrap();

    // Compute a bfs starting from `node`
    let mut queue = VecDeque::with_capacity(MAX_NODES);
    queue.push_back(node);
    visited.insert(node);

    // In the queue there are only nodes which have not been visited
    while let Some(node) = queue.pop_front() {
        n_visited += 1;

        for neighbor in graph[node].ones() {
            if !visited.contains(neighbor) {
                visited.insert(neighbor);
                queue.push_back(neighbor);
            }
        }
    }

    // The graph is connected iff it has only one connected component
    if n_visited == n_nodes {
        Ok(())
    } else {
        Err("The network topology is not connected".to_string().into())
    }
}

/// Checks if all clients and servers are at the edge of the newtork.   
///
/// # Parameters
/// - `graph` - Graph of the network.
/// - `drone_ids` - Set of all the drone ids in the network.
/// - `n_drones` - Number of drones in the network.
///
/// # Performance
/// - `O(n + m)`, where `n` and `m` are the number of nodes and edges in the network, respectively.  
fn validate_edges_clients_servers(
    graph: &Graph,
    drone_ids: &FixedBitSet,
    n_nodes: usize,
    n_drones: usize,
) -> Result<(), String> {
    // There are no nodes in the network
    if n_nodes == 0 {
        return Ok(());
    }
    // There are no drones in the network
    if n_drones == 0 {
        return Ok(());
    }

    let mut visited = FixedBitSet::with_capacity(MAX_NODES);
    let mut n_visited = 0;
    let drone = drone_ids.ones().next().unwrap();

    // Compute a bfs starting from `drone`
    let mut queue = VecDeque::with_capacity(MAX_NODES);
    queue.push_back(drone);
    visited.insert(drone);

    while let Some(node) = queue.pop_front() {
        n_visited += 1;

        for neighbor in graph[node].ones() {
            if !drone_ids.contains(neighbor) {
                continue;
            }
            if !visited.contains(neighbor) {
                visited.insert(neighbor);
                queue.push_back(neighbor);
            }
        }
    }

    if n_visited == n_drones {
        Ok(())
    } else {
        Err("Clients and servers are not all on the edge of the network"
            .to_string()
            .into())
    }
}
