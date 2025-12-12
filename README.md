# Rust + ROS 2 (Jazzy) Pub/Sub Demo

This repository is a small demo showing how to build and run Rust-based ROS 2 nodes (publisher and subscriber) against ROS 2 Jazzy. It contains example Rust packages and convenience scripts to prepare the environment, build the workspace, and run the demo both from the command line and from Visual Studio Code's debugger.

**Table Of Contents**
- [Project](#project)
- [Prerequisites](#prerequisites)
- [Quick Setup](#quick-setup)
- [Build](#build)
- [Run](#run)
- [VS Code Debugging](#vs-code-debugging)
- [VS Code Extensions (recommended)](#vs-code-extensions-recommended)
- [Architecture & Structure](#architecture--structure)
- [Troubleshooting](#troubleshooting)

## Project
- **Description**: Demonstrates ROS 2 publisher/subscriber implemented in Rust using the `r2r` / `rclrs` (Rust client library) bindings. Use this to learn Rust + ROS 2 integration or as a starting point for your own Rust nodes.

## Prerequisites
- **ROS 2 Jazzy**: Installed on the host (commonly in `/opt/ros/jazzy`).
- **Rust toolchain**: `rustc`, `cargo` (use `rustup` to install/manage toolchains).
- **colcon**: Used to build the workspace (and the Rust packages wrapped for ROS 2).
- **Development tools**: Recommended — Visual Studio Code, CMake, build-essential packages.

## Quick Setup
- The repository includes helper scripts to prepare the environment. From the repository root run:

```bash
bash ./setup_ros2_jazzy.sh
```

- Additional setup scripts in the repo (run if you need them):

```bash
bash ./setup_rust.sh        # install rustup and toolchain
bash ./setup_git_user.sh    # configure git user
```

- `setup_rust.sh` installs `rustup` and sets up `~/.cargo/bin` in the PATH. `setup_git_user.sh` sets your global git user info used for commits.

If you prefer to source ROS manually, run:

```bash
source /opt/ros/jazzy/setup.bash
```

After building you'll need to source the workspace overlay:

```bash
source install/setup.bash
```

## Build
- Build the whole workspace (recommended):

```bash
# from repository root
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

- Build only the Rust packages (example):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rust_publisher rust_subscriber
source install/setup.bash
```

### How colcon builds Rust packages in this workspace
Rust ROS 2 packages are built through a CMake wrapper that invokes Cargo. The typical flow:

- A package contains a `CMakeLists.txt` which calls into a helper/CMake macro that runs `cargo build` for the Rust crate.
- `colcon` runs the CMake build, which in turn builds the Rust artifacts and installs them into the ROS `install/` overlay.

This approach allows ROS 2 tooling (message generation, ament/CMake install paths, environment hooks) to interoperate with Rust's `cargo` build system.

## Run
- Run using `ros2 run` (when the package exports the executable):

```bash
# Example (replace names if different in this repo)
ros2 run rust_publisher rust_publisher
ros2 run rust_subscriber rust_subscriber
```

- Alternatively run the built executables directly from the install folder:

```bash
ls install/<package>/lib/<package>/
./install/<package>/lib/<package>/<executable>
```

- If you're experimenting, open separate terminals for publisher and subscriber and ensure `source install/setup.bash` has been run in each terminal.

## VS Code Debugging
- This repository contains a VS Code launch configuration at `/.vscode/launch.json` to help start and attach the debugger to built Rust nodes. To debug effectively:
  - Build the workspace: `colcon build` and `source install/setup.bash`.
  - Open the project in VS Code.
  - Open the Run and Debug view and choose an appropriate configuration (Rust node) from the dropdown.

- Tips for a reliable debug session:
  - Ensure the debug configuration runs the binary from the `install/` tree or sets `cwd` to the package directory so relative paths resolve correctly.
  - Make sure the environment includes ROS 2: either launch VS Code from a shell where `source /opt/ros/jazzy/setup.bash` and `source install/setup.bash` were executed, or add environment variables in `launch.json` (for example set `LD_LIBRARY_PATH` to include `${workspaceFolder}/install/lib`).

## VS Code Extensions (recommended)
- `Rust Extension Pack` — provides the Rust language support, formatting, and useful tooling for editing and debugging Rust.
- `CMake Tools` — makes editing, configuring and building CMake/CMakeLists-based projects (such as ROS 2 packages) inside VS Code much easier.

## Architecture & Structure
This workspace follows the standard ROS 2 colcon workspace layout with Rust packages built via Cargo wrapped by CMake.

Example directory layout:

```
rust_ros2_demo_ws/
├─ src/
│  ├─ rust_publisher/      # ROS2 package (CMakeLists + Rust crate)
│  │  └─ src/
│  │     └─ main.rs
│  ├─ rust_subscriber/
│  │  └─ src/
│  │     └─ main.rs
├─ install/                 # populated by `colcon build` (install overlay)
├─ build/                   # build artifacts
├─ log/                     # build/run logs
├─ setup_ros2_jazzy.sh      # helper: source ROS 2 and workspace helpers
├─ setup_rust.sh            # helper: install rustup and toolchain
├─ setup_git_user.sh        # helper: configure git user
├─ .vscode/launch.json      # VS Code debug configs
├─ README.md
```

### Rust ROS 2 node model (brief)
- Rust subscriber nodes commonly use a "pull" model: the subscription returns a `Stream` (or an async iterator) and your task `await`s `next()` to receive messages. This is in contrast to the callback-based "push" model often used in C++ or Python ROS 2 examples where the middleware invokes a callback when messages arrive.

- Advantages of the pull model in Rust:
  - Integrates naturally with async runtimes and `Stream` combinators.
  - Makes backpressure and structured async control easier to express.

### Runtime & threading recommendations
- Use a multi-threaded Tokio runtime for Rust ROS 2 nodes and run a dedicated spin task that calls `node.spin_once(...)` in a loop. Reasons:
  - Tokio's multithreaded runtime allows publisher/subscriber tasks and other async work to run concurrently without blocking the spin loop.
  - A dedicated spin task keeps the ROS 2 event processing responsive and makes handling `Ctrl+C` clean (the spin task can be cancelled via a token).

In this demo the publisher and subscriber use:
- `Arc<Mutex<Node>>` for shared access to the `Node` from multiple tasks.
- `tokio_util::sync::CancellationToken` to request shutdown and let tasks stop gracefully.

## Troubleshooting
- If `colcon build` fails:
  - Confirm ROS 2 Jazzy is installed and `source /opt/ros/jazzy/setup.bash` was run.
  - Verify Rust toolchain is available (`rustc --version`, `cargo --version`).
- If nodes fail to find message types or libraries at runtime, ensure `source install/setup.bash` is executed in the running shell so the overlayed environment is active.
- If debugger cannot attach or breakpoints are ignored, ensure the binaries were built with debug symbols (default `cargo build` in debug mode) and that the debugger configuration points at the correct executable path.


