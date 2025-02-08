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

    let network = network_validate("src/config.toml")?;

    Ok(())
}
