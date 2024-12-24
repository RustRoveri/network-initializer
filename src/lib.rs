use std::env;

use utils::network_init;

pub mod utils;

#[test]
fn test_0() -> Result<(), String> {
    match env::current_dir() {
        Ok(path) => println!("Current working directory: {}", path.display()),
        Err(e) => println!("Error getting current directory: {}", e),
    }
    let network = network_init("network-initializer/src/config.toml")?;

    for v in network.client {
        println!("{:?}", v);
    }

    for v in network.server {
        println!("{:?}", v);
    }
    
    for v in network.drone {
        println!("{:?}", v);
    }

    Ok(())
}