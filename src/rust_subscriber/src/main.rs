use r2r::*;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use futures_util::StreamExt;
use tokio_util::sync::CancellationToken;

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS2 context
    let ctx = Context::create()?;
    let node = Arc::new(Mutex::new(Node::create(ctx, "rust_listener", "")?));
    
    println!("Rust Subscriber Node Started");
    println!("Listening on /chatter\n");

    let cancel = CancellationToken::new();
    
    // Create subscriber
    let mut subscriber = {
        let mut n = node.lock().unwrap();
        n.subscribe::<std_msgs::msg::String>("/chatter", QosProfile::default())?
    };
    
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

    // Dedicated spin task
    let node_spin = node.clone();
    let cancel_spin = cancel.clone();
    tokio::task::spawn(async move {
        loop {
            tokio::select! {
                biased;
                
                _ = cancel_spin.cancelled() => {
                    println!("Spin task stopping...");
                    break;
                }
                
                _ = tokio::time::sleep(Duration::from_millis(0)) => {
                    let mut n = node_spin.lock().unwrap();
                    n.spin_once(Duration::from_millis(100));
                }
            }
        }
    });
    
    // Main spin loop
    println!("Node running. Press Ctrl+C to stop.\n");
    
    // Wait for Ctrl+C
    tokio::signal::ctrl_c().await?;
    println!("\nShutting down subscriber...");
    
    cancel.cancel();
    tokio::time::sleep(Duration::from_millis(50)).await;
    
    println!("Subscriber stopped.");
    Ok(())
}
