use fixedbitset::FixedBitSet;
use rust_roveri_api::MAX_NODES;
use std::{collections::VecDeque, fs};
use wg_2024::config::{Client, Config, Drone, Server};

type Graph = [FixedBitSet; MAX_NODES];

/// Reads and validates the network configuration file.   
///
/// This function attempts to read the configuration file from the given `file_path`,
/// deserializes its contents as TOML into a `Config` instance, and then verifies that
/// the encoded topology is valid.
///
/// # Parameters
/// - `file_path`: The path of the configuration file.
///
/// Returns the configuration, as `Config`, if the configuration file provided is valid, an error otherwise.
pub fn network_validate(file_path: &str) -> Result<Config, String> {
    // Read the configuration file as a string.
    let config_data = fs::read_to_string(file_path)
        .map_err(|_| "Unable to read configuration file".to_string())?;
    // Deserialize the TOML data into a Config.
    let config: Config =
        toml::from_str(&config_data).map_err(|e| format!("Failed to deserialize TOML: {}", e))?;

    // Validate the configuration.
    validate_config(&config)?;

    Ok(config)
}

/// Validates the entire network configuration.
///
/// This function checks that:
/// - Each drone, client, and server is valid individually.
/// - There are no duplicate node IDs across all node types.
/// - Every client and server connects only to drones.
/// - The constructed network graph is bidirectional, connected,
///   and clients/servers are at the network edge.
///
/// # Parameters
/// - `config`: A reference to the network configuration.
///
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n + m)`, where `n` is the number of nodes and `m` is the number of edges.
fn validate_config(config: &Config) -> Result<(), String> {
    let mut n_nodes = 0;
    let mut node_ids = FixedBitSet::with_capacity(MAX_NODES);

    // Validate drones.
    for drone in &config.drone {
        validate_drone(drone)?;
        if node_ids.contains(drone.id as usize) {
            return Err(format!("Duplicate node ID found: [{}]", drone.id));
        } else {
            node_ids.insert(drone.id as usize);
            n_nodes += 1;
        }
    }
    let drone_ids = node_ids.clone();
    let n_drones = n_nodes;

    // Validate clients.
    for client in &config.client {
        validate_client(client)?;
        if node_ids.contains(client.id as usize) {
            return Err(format!("Duplicate node ID found: [{}]", client.id));
        } else {
            node_ids.insert(client.id as usize);
            n_nodes += 1;
        }
    }

    // Validate servers.
    for server in &config.server {
        validate_server(server)?;
        if node_ids.contains(server.id as usize) {
            return Err(format!("Duplicate node ID found: [{}]", server.id));
        } else {
            node_ids.insert(server.id as usize);
            n_nodes += 1;
        }
    }

    // Confirm that the total node count matches.
    assert_eq!(
        n_nodes,
        config.drone.len() + config.client.len() + config.server.len()
    );

    // Check that all clients and servers connect only to drones.
    validate_all_neighbors_are_drones(config, &drone_ids)?;

    let mut graph: Graph = std::array::from_fn(|_| FixedBitSet::with_capacity(MAX_NODES));
    compute_init_graph(&mut graph, config);
    validate_bidirectional_graph(&graph, &node_ids)?;
    validate_connected_graph(&graph, &node_ids, n_nodes)?;
    validate_edges_clients_servers(&graph, &drone_ids, n_nodes, n_drones)?;

    Ok(())
}

/// Validates a drone's configuration.
///
/// Ensures that the drone's packet drop rate (PDR) is between 0 and 1,
/// that the drone is not connected to itself, and that there are no duplicate entries
/// in its neighbor list.
///
/// # Parameters
/// - `drone`: The drone to validate.
///
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n)`, where `n` is the number of neighbors.
fn validate_drone(drone: &Drone) -> Result<(), String> {
    if drone.pdr < 0_f32 || drone.pdr > 1_f32 {
        return Err(format!(
            "Invalid PDR for drone [{}]: {}",
            drone.id, drone.pdr
        ));
    }
    let mut set = FixedBitSet::with_capacity(MAX_NODES);
    for connected_id in &drone.connected_node_ids {
        if *connected_id == drone.id {
            return Err(format!("Drone [{}] is connected to itself", drone.id));
        }
        if set.contains(*connected_id as usize) {
            return Err(format!(
                "Drone [{}] has duplicate neighbor [{}]",
                drone.id, *connected_id
            ));
        }
        set.insert(*connected_id as usize);
    }
    Ok(())
}

/// Validates a client's configuration.
///
/// Checks that the client is connected to at least one and at most two drones,
/// and that its neighbor list does not contain self-connections or duplicates.
///
/// # Parameters
/// - `client`: The client to validate.
///
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n)`, where `n` is the number of neighbors.
fn validate_client(client: &Client) -> Result<(), String> {
    if client.connected_drone_ids.is_empty() {
        return Err(format!("Client [{}] is connected to 0 drones", client.id));
    }
    if client.connected_drone_ids.len() > 2 {
        return Err(format!("Client [{}] has more than 2 neighbors", client.id));
    }
    let mut set = FixedBitSet::with_capacity(MAX_NODES);
    for connected_id in &client.connected_drone_ids {
        if *connected_id == client.id {
            return Err(format!("Client [{}] is connected to itself", client.id));
        }
        if set.contains(*connected_id as usize) {
            return Err(format!(
                "Client [{}] has duplicate neighbor [{}]",
                client.id, *connected_id
            ));
        }
        set.insert(*connected_id as usize);
    }
    Ok(())
}

/// Validates a server's configuration.
///
/// Ensures that the server is connected to at least two drones, is not self-connected,
/// and does not have duplicate neighbors.
///
/// # Parameters
/// - `server`: The server to validate.
///
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n)`, where `n` is the number of neighbors.
fn validate_server(server: &Server) -> Result<(), String> {
    if server.connected_drone_ids.len() < 2 {
        return Err(format!("Server [{}] has less than 2 neighbors", server.id));
    }
    let mut set = FixedBitSet::with_capacity(MAX_NODES);
    for connected_id in &server.connected_drone_ids {
        if *connected_id == server.id {
            return Err(format!("Server [{}] is connected to itself", server.id));
        }
        if set.contains(*connected_id as usize) {
            return Err(format!(
                "Server [{}] has duplicate neighbor [{}]",
                server.id, *connected_id
            ));
        }
        set.insert(*connected_id as usize);
    }
    Ok(())
}

/// Validates that all neighbors specified for clients and servers are drones.
///
/// Iterates over every client and server in the configuration and ensures that every
/// neighbor ID appears in the provided set of drone IDs.
///
/// # Parameters
/// - `config`: The network configuration.
/// - `drone_ids`: A FixedBitSet containing all drone IDs.
///
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n + m)`, where `n` is the number of nodes and `m` is the number of edges.
fn validate_all_neighbors_are_drones(
    config: &Config,
    drone_ids: &FixedBitSet,
) -> Result<(), String> {
    for client in &config.client {
        for id in &client.connected_drone_ids {
            if !drone_ids.contains(*id as usize) {
                return Err(format!(
                    "Client [{}] is connected to [{}], which is not a drone",
                    client.id, *id
                ));
            }
        }
    }
    for server in &config.server {
        for id in &server.connected_drone_ids {
            if !drone_ids.contains(*id as usize) {
                return Err(format!(
                    "Server [{}] is connected to [{}], which is not a drone",
                    server.id, *id
                ));
            }
        }
    }
    Ok(())
}

/// Builds the initial network graph from the configuration.
///
/// The graph is represented as an array of FixedBitSet (one per node), where each FixedBitSet
/// contains the neighbor IDs for that node.
///
/// # Parameters
/// - `graph`: A mutable reference to the graph to be constructed.
/// - `config`: The network configuration.
///
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n + m)`, where `n` is the number of nodes and `m` is the number of edges.
fn compute_init_graph(graph: &mut Graph, config: &Config) {
    for drone in &config.drone {
        for id in &drone.connected_node_ids {
            graph[drone.id as usize].insert(*id as usize);
        }
    }
    for client in &config.client {
        for id in &client.connected_drone_ids {
            graph[client.id as usize].insert(*id as usize);
        }
    }
    for server in &config.server {
        for id in &server.connected_drone_ids {
            graph[server.id as usize].insert(*id as usize);
        }
    }
}

/// Validates that the network graph is bidirectional.
///
/// For every edge from node A to node B in the graph, this function ensures that node B also has an edge back to A.
///
/// # Parameters
/// - `graph`: The network graph.
/// - `node_ids`: A FixedBitSet containing all valid node IDs.
///
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n + m)`, where `n` is the number of nodes and `m` is the number of edges.
fn validate_bidirectional_graph(graph: &Graph, node_ids: &FixedBitSet) -> Result<(), String> {
    for node in node_ids.ones() {
        for id in graph[node].ones() {
            if !node_ids.contains(id) {
                return Err(format!(
                    "Node [{}] has [{}] as neighbor, which does not exist in the topology.",
                    node, id
                ));
            }
            if !graph[id].contains(node) {
                return Err(format!(
                    "The topology is not bidirectional: node [{}] is reachable from [{}], but not vice versa.",
                    id, node
                ));
            }
        }
    }
    Ok(())
}

/// Validates that the network graph is connected.
///
/// Performs a breadth-first search (BFS) starting from an arbitrary node and checks that every
/// node in the topology is reachable.
///
/// # Parameters
/// - `graph`: The network graph.
/// - `node_ids`: A FixedBitSet containing all valid node IDs.
/// - `n_nodes`: The total number of nodes in the network.
///  
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n + m)`, where `n` is the number of nodes and `m` is the number of edges.
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
    let start_node = node_ids.ones().next().unwrap();

    let mut queue = VecDeque::with_capacity(MAX_NODES);
    queue.push_back(start_node);
    visited.insert(start_node);

    while let Some(node) = queue.pop_front() {
        n_visited += 1;
        for neighbor in graph[node].ones() {
            if !visited.contains(neighbor) {
                visited.insert(neighbor);
                queue.push_back(neighbor);
            }
        }
    }

    if n_visited == n_nodes {
        Ok(())
    } else {
        Err("The network topology is not connected".to_string())
    }
}

/// Validates that all clients and servers are on the edge of the network,
/// meaning they are connected only to drones.
///
/// This function performs a BFS starting from an arbitrary drone and verifies that every drone
/// is reachable, implying that non-drone nodes (clients/servers) are not forming internal connections.
///
/// # Parameters
/// - `graph`: The network graph.
/// - `drone_ids`: A FixedBitSet containing the IDs of all drones.
/// - `n_nodes`: The total number of nodes in the network.
/// - `n_drones`: The number of drones in the network.
///
/// Returns an error if the checks are not passed.
///
/// # Performance
/// `O(n + m)`, where `n` is the number of nodes and `m` is the number of edges.
fn validate_edges_clients_servers(
    graph: &Graph,
    drone_ids: &FixedBitSet,
    n_nodes: usize,
    n_drones: usize,
) -> Result<(), String> {
    if n_nodes == 0 || n_drones == 0 {
        return Ok(());
    }

    let mut visited = FixedBitSet::with_capacity(MAX_NODES);
    let mut n_visited = 0;
    let start_drone = drone_ids.ones().next().unwrap();

    let mut queue = VecDeque::with_capacity(MAX_NODES);
    queue.push_back(start_drone);
    visited.insert(start_drone);

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
        Err("Clients and servers are not all on the edge of the network".to_string())
    }
}
