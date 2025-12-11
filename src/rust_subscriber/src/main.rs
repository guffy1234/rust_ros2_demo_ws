use r2r::*;
use std::time::Duration;
use futures_util::StreamExt;


#[tokio::main]
async fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS2 context
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "rust_listener", "")?;
    
    println!("Rust Subscriber Node Started");
    println!("Listening on /chatter\n");
    
    // Create subscriber
    let mut subscriber = node.subscribe::<std_msgs::msg::String>(
        "/chatter",
        QosProfile::default()
    )?;
    
    // Spawn subscription task
    tokio::task::spawn(async move {
        loop {
            match subscriber.next().await {
                Some(msg) => {
                    println!("Received: '{}'", msg.data);
                }
                None => {
                    eprintln!("Subscriber stream ended");
                    break;
                }
            }
        }
    });
    
    // Main spin loop
    println!("Node running. Press Ctrl+C to stop.\n");
    
    loop {
        tokio::select! {
            biased;
            
            _ = tokio::signal::ctrl_c() => {
                println!("\nShutting down subscriber...");
                break;
            }
            
            _ = tokio::time::sleep(Duration::from_millis(0)) => {
                node.spin_once(Duration::from_millis(100));
            }
        }
    }
    
    println!("Subscriber stopped.");
    Ok(())
}
