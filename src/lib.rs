//! # Network Initializer Crate
//!
//! This crate is responsible for validating and initializing a network topology based on a user-supplied
//! configuration file. It provides functions to:
//!
//! - **Validate the Configuration:**  
//!   The function [`network_validate`] reads a TOML configuration file, deserializes it into a `Config` structure, and performs   
//!   comprehensive checks. These checks ensure that:
//!     - Each drone, client, and server has valid parameters (e.g., proper packet drop rate, no self-loops,
//!       and no duplicate neighbor entries).
//!     - There are no duplicate node IDs across the entire network.
//!     - Every client and server is connected only to drones.
//!     - The overall network graph is bidirectional and connected.
//!
//! - **Initialize the Network:**  
//!   The function [`network_init`] builds the network topology by:
//!     - Creating arrays for node types (as `(NodeType, FixedBitSet)`), command channels, and packet send channels.
//!     - Setting up unbounded channels for drones, clients, and servers.
//!     - Constructing distribution data for node types (drones, clients, servers).
//!     - Spawning threads for each node (using functions such as `factory_drone` for drones, and similar
//!       routines for clients and servers).
//!     - Updating the topology graph by inserting neighbor edges and sending initial commands to add links.
//!     - Assembling all of the data into a `NetworkInitData` structure, which is then used by both the simulation
//!       controller and the GUI.
//!
//! ## Overview
//!
//! The typical workflow for using this crate is as follows:
//!
//! 1. **Configuration Reading and Validation:**  
//!    Call `network_validate` with the path to your configuration file. If the configuration is valid, a `Config`
//!    object is returned; otherwise, an error is produced detailing the validation issues.
//!
//! 2. **Network Initialization:**  
//!    Pass the validated `Config` to `network_init` to spawn node threads and build the necessary data structures.
//!    The returned `NetworkInitData` encapsulates the complete network topology, communication channels, and
//!    distribution information.
//!
//! 3. **Integration:**  
//!    The `NetworkInitData` is y the simulatithen used bon controller and the GUI to reflect the network as specified
//!    by the user.
//!
//! ## Performance
//!
//! Most validation routines in this crate operate in `O(n + m)` time, where `n` is the number of nodes and `m` is the
//! number of edges in the network. This ensures that the network initializer scales linearly with the size of the network.
#![allow(unused_imports)]
use init::network_init;
use std::env;
use validate::network_validate;

pub mod init;
pub mod validate;

#[test]
fn test_0() -> Result<(), String> {
    match env::current_dir() {
        Ok(path) => println!("Current working directory: {}", path.display()),
        Err(e) => println!("Error getting current directory: {}", e),
    }

    let config = network_validate("src/config.toml")?;
    let _res = network_init(&config);

    Ok(())
}
