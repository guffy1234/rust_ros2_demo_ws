use r2r::*;
use std::time::Duration;

#[tokio::main]
async fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS2 context
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "rust_talker", "")?;
    
    println!("Rust Publisher Node Started");
    println!("Publishing to /chatter at 1 Hz");
    
    // Create publisher
    let publisher = node.create_publisher::<std_msgs::msg::String>(
        "/chatter",
        QosProfile::default()
    )?;
    
    // Spawn publishing task
    tokio::task::spawn(async move {
        let mut count = 0;
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        
        loop {
            interval.tick().await;
            
            let mut msg = std_msgs::msg::String::default();
            msg.data = format!("Hello from Rust! Count: {}", count);
            
            match publisher.publish(&msg) {
                Ok(_) => println!("Published: '{}'", msg.data),
                Err(e) => eprintln!("Failed to publish: {:?}", e),
            }
            
            count += 1;
        }
    });
    
    // Main spin loop
    println!("Node running. Press Ctrl+C to stop.\n");
    
    loop {
        tokio::select! {
            biased;
            
            _ = tokio::signal::ctrl_c() => {
                println!("\nShutting down publisher...");
                break;
            }
            
            _ = tokio::time::sleep(Duration::from_millis(0)) => {
                node.spin_once(Duration::from_millis(100));
            }
        }
    }
    
    println!("Publisher stopped.");
    Ok(())
}
