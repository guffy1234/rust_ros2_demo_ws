use r2r::*;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio_util::sync::CancellationToken;

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS2 context
    let ctx = Context::create()?;
    let node = Arc::new(Mutex::new(Node::create(ctx, "rust_talker", "")?));

    println!("Rust Publisher Node Started");
    println!("Publishing to /chatter at 1 Hz");

    // Cancellation token for graceful shutdown
    let cancel = CancellationToken::new();

    // Create publisher by locking the node briefly
    let publisher = {
        let mut n = node.lock().unwrap();
        n.create_publisher::<std_msgs::msg::String>("/chatter", QosProfile::default())?
    };

    // Spawn publishing task (owns the publisher)
    let cancel_pub = cancel.clone();
    tokio::task::spawn(async move {
        let mut count: u64 = 0;
        let mut interval = tokio::time::interval(Duration::from_secs(1));

        loop {
            tokio::select! {
                biased;
                _ = cancel_pub.cancelled() => {
                    println!("Publisher task stopping...");
                    break;
                }
                _ = interval.tick() => {
                    let mut msg = std_msgs::msg::String::default();
                    msg.data = format!("Hello from Rust! Count: {}", count);

                    match publisher.publish(&msg) {
                        Ok(_) => println!("Published: '{}'", msg.data),
                        Err(e) => eprintln!("Failed to publish: {:?}", e),
                    }

                    count = count.wrapping_add(1);
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

    println!("Node running. Press Ctrl+C to stop.\n");

    // Wait for Ctrl+C
    tokio::signal::ctrl_c().await?;
    println!("\nShutting down publisher and spin task...");

    // Signal cancellation and give tasks a moment to stop
    cancel.cancel();
    tokio::time::sleep(Duration::from_millis(50)).await;

    println!("Publisher stopped.");
    Ok(())
}
