use std::{collections::HashSet, error::Error, fs};

use ahash::{AHashMap, AHashSet};
use wg_2024::{
    config::{Client, Config, Drone, Server},
    network::NodeId,
};

type Graph = AHashMap<NodeId, AHashSet<NodeId>>;

/// API to compute the initialization of the network.   
///
/// The function assumes that the directory from where the program is called, ie the current working directory,    
/// contains the `config/config.toml` file.   
///
/// Returns `Ok` if the initialization process is successful, `Err` with an error message if an error occurs.
pub fn network_init() -> Result<Config, Box<dyn Error>> {
    // Deserializing the config file
    let config_data =
        fs::read_to_string("/home/michele3/Documents/uniTN/III/advanced-programming/RustRoveri/network-initializer/src/config.toml")
        .map_err(|_| "Unable to read file config.toml".to_string())?;
    let config: Config =
        toml::from_str(&config_data).map_err(|e| format!("Failed to deserialize TOML: {}", e))?;

    // Validate the configuration
    validate_config(&config)?;

    Ok(config)
}

fn validate_config(config: &Config) -> Result<(), Box<dyn Error>> {
    // `node_ids` as a set of `MyPair` is necessary for later: The user could provide ids which are not indexed,
    // but they would still be valid. Ex:
    let mut node_ids = AHashSet::new();

    // Validate drones
    for drone in &config.drone {
        validate_drone(drone)?;
        if !node_ids.insert(drone.id) {
            return Err(format!("Duplicate node ID found: {}", drone.id).into());
        }
    }
    let mut drone_ids = node_ids.clone();
    // From now on `drone_ids` is never modified. Optimizes memory
    drone_ids.shrink_to_fit();

    // Validate clients
    for client in &config.client {
        validate_client(client)?;
        if !node_ids.insert(client.id) {
            return Err(format!("Duplicate node ID found: {}", client.id).into());
        }
    }

    // Validate servers
    for server in &config.server {
        validate_server(server)?;
        if !node_ids.insert(server.id) {
            return Err(format!("Duplicate node ID found: {}", server.id).into());
        }
    }
    // From now on `node_ids` is never modified. Optimizes memory
    node_ids.shrink_to_fit();

    eprintln!("{:?}", node_ids);

    assert_eq!(
        node_ids.len(),
        config.drone.len() + config.client.len() + config.server.len()
    );

    // Check for each client/server if its neighbors are all drones
    validate_all_neighbors_are_drones(config, &drone_ids)?;

    // Storing the graph as `Vec<AHashSet<NodeId>>` is good for time efficiency, because we still have to checks two things:
    //   i) if the graph is bidirectional (AHashSet is useful here, so that we can optimize the algorithm)
    //   ii) if the graph is "hyper"-connected, ie if it is connected without considering the clients and servers.
    let mut graph: Graph = AHashMap::new();
    compute_init_graph(&mut graph, config);
    // From now on `graph` is never modified. Optimizes memory
    graph.shrink_to_fit();

    eprintln!("{:?}", graph);

    validate_bidirectional_graph(&graph)?;

    validate_connected_graph(&graph)?;

    validate_edges_clients_servers(&graph, &drone_ids)?;

    // If the PC arrives here, then the network initialization is successful
    println!("{:?}", config);

    Ok(())
}

fn validate_drone(drone: &Drone) -> Result<(), Box<dyn Error>> {
    // Checks if the PDR is in bounds
    if drone.pdr < 0_f32 || drone.pdr > 1_f32 {
        return Err(format!("Invalid PDR for drone {}: {}", drone.id, drone.pdr).into());
    }

    // Checks if the drone itself is in the list of its neighbors and if there are not duplicates in the list
    let mut connected_set = HashSet::new();
    for &connected_id in &drone.connected_node_ids {
        if connected_id == drone.id {
            return Err(format!("Drone {} is connected to itself", drone.id).into());
        }
        if !connected_set.insert(connected_id) {
            return Err(format!(
                "Duplicate connection for drone {}: {}",
                drone.id, connected_id
            )
            .into());
        }
    }

    Ok(())
}

fn validate_client(client: &Client) -> Result<(), Box<dyn Error>> {
    // Checks if the server is connected to 0 drones
    if client.connected_drone_ids.is_empty() {
        return Err(format!("Client {} is connected to 0 drones", client.id).into());
    }

    // Checks if the server is connected to n > 2 drones
    if client.connected_drone_ids.len() > 2 {
        return Err(format!("Client {} has more than 2 neighbors", client.id).into());
    }

    // Checks if the client itself is in the list of its neighbors and if there are not duplicates in the list
    let mut connected_set = AHashSet::new();
    for connected_id in &client.connected_drone_ids {
        if *connected_id == client.id {
            return Err(format!("Client {} is connected to itself", client.id).into());
        }
        if !connected_set.insert(connected_id) {
            return Err(format!(
                "Duplicate connection for client {}: {}",
                client.id, connected_id
            )
            .into());
        }
    }

    Ok(())
}

fn validate_server(server: &Server) -> Result<(), Box<dyn Error>> {
    // Checks if the server has at least 2 neighbors
    if server.connected_drone_ids.len() < 2 {
        return Err(format!("Server {} is connected to fewer than 2 drones", server.id).into());
    }

    // Checks if the client itself is in the list of its neighbors and if there are not duplicates in the list
    let mut connected_set = AHashSet::new();
    for connected_id in &server.connected_drone_ids {
        if *connected_id == server.id {
            return Err(format!("Server {} is connected to itself", server.id).into());
        }
        if !connected_set.insert(connected_id) {
            return Err(format!(
                "Duplicate connection in server {}: {}",
                server.id, connected_id
            )
            .into());
        }
    }

    Ok(())
}

/// For every server and client checks if their neighbors list contains only ids of drones.   
/// `O(n + m)`
fn validate_all_neighbors_are_drones(
    config: &Config,
    drone_ids: &AHashSet<NodeId>,
) -> Result<(), Box<dyn Error>> {
    // Client check
    for client in &config.client {
        for id in &client.connected_drone_ids {
            if !drone_ids.contains(id) {
                return Err(format!(
                    "Client {} is connected to a node which is not a drone",
                    client.id
                )
                .into());
            }
        }
    }

    // Server check
    for server in &config.server {
        for id in &server.connected_drone_ids {
            if !drone_ids.contains(id) {
                return Err(format!(
                    "Server {} is connected to a node which is not a drone",
                    server.id
                )
                .into());
            }
        }
    }

    Ok(())
}

/// `O(n + m)`
fn compute_init_graph(graph: &mut Graph, config: &Config) {
    // Stores the drones and their connections in the graph
    for drone in &config.drone {
        for id in &drone.connected_node_ids {
            graph
                .entry(drone.id)
                .and_modify(|set| {
                    set.insert(*id);
                })
                .or_insert_with(|| {
                    let mut set = AHashSet::new();
                    set.insert(*id);
                    set
                });
        }
    }

    // Stores the clients and their connections in the graph
    for client in &config.client {
        for id in &client.connected_drone_ids {
            graph
                .entry(client.id)
                .and_modify(|set| {
                    set.insert(*id);
                })
                .or_insert_with(|| {
                    let mut set = AHashSet::new();
                    set.insert(*id);
                    set
                });
        }
    }

    // Stores the servers and their connections in the graph
    for server in &config.server {
        for id in &server.connected_drone_ids {
            graph
                .entry(server.id)
                .and_modify(|set| {
                    set.insert(*id);
                })
                .or_insert_with(|| {
                    let mut set = AHashSet::new();
                    set.insert(*id);
                    set
                });
        }
    }
}

/// `O(n + m)`
fn validate_bidirectional_graph(graph: &Graph) -> Result<(), Box<dyn Error>> {
    for node in graph.keys() {
        // We are considering the edge [node] -> [id]
        for id in graph.get(node).unwrap() {
            // Edge case: checks if the node id is in the topology, ie if the user has specified it as a node in the config file, O(1)
            if !graph.contains_key(id) {
                return Err(format!(
                    "Node {} has as neighbor a non existent node in the topology: {}",
                    node, id
                )
                .into());
            }

            // Checks if the edge [id] -> [node], O(1)
            if !graph.get(id).unwrap().contains(node) {
                return Err(format!(
                    "The topology does not represent a bidirectional graph:\n
                    node {} is reachable from {}, but not viceversa",
                    id, node
                )
                .into());
            }
        }
    }

    Ok(())
}

/// `O(n + m)`
fn validate_connected_graph(graph: &Graph) -> Result<(), Box<dyn Error>> {
    if graph.len() == 0 {
        return Ok(());
    }

    let mut visited = AHashSet::new();
    let node = graph.keys().next().unwrap();
    compute_dfs(graph, *node, &mut visited);

    // Only one connected component
    if graph.len() == visited.len() {
        Ok(())
    } else {
        Err("The network topology is not entirely bidirectional"
            .to_string()
            .into())
    }
}

/// `O(n + m)`
fn compute_dfs(graph: &Graph, node: NodeId, visited: &mut AHashSet<NodeId>) {
    visited.insert(node);

    for v in graph.get(&node).unwrap() {
        if !visited.contains(v) {
            compute_dfs(graph, *v, visited);
        }
    }
}

/// `O(n + m)`
fn validate_edges_clients_servers(
    graph: &Graph,
    drone_ids: &AHashSet<NodeId>,
) -> Result<(), Box<dyn Error>> {
    if graph.len() == 0 {
        return Ok(());
    }

    let mut visited = AHashSet::new();
    let node = drone_ids.iter().next().unwrap();
    compute_dfs_(graph, *node, &mut visited, drone_ids);

    // Only one connected component
    if drone_ids.len() == visited.len() {
        Ok(())
    } else {
        Err(
            "Clients and servers are not completely at the edges of the network topology"
                .to_string()
                .into(),
        )
    }
}

/// `O(n + m)`
fn compute_dfs_(
    graph: &Graph,
    node: NodeId,
    visited: &mut AHashSet<NodeId>,
    drones_ids: &AHashSet<NodeId>,
) {
    visited.insert(node);

    for v in graph.get(&node).unwrap() {
        if !visited.contains(v) && drones_ids.contains(v) {
            compute_dfs_(graph, *v, visited, drones_ids);
        }
    }
}
