# Rust + ROS 2 (Jazzy) Pub/Sub Demo

This repository is a small demo showing how to build and run Rust-based ROS 2 nodes (publisher and subscriber) against ROS 2 Jazzy. It contains example Rust packages and convenience scripts to prepare the environment, build the workspace, and run the demo both from the command line and from Visual Studio Code's debugger.

**Table Of Contents**
- **Project**: Short description of the demo.
- **Prerequisites**: System requirements and tools.
- **Quick Setup**: Use provided scripts to prepare the environment.
- **Build**: How to build the workspace (colcon) or individual Rust packages.
- **Run**: Run nodes from the command line or find built binaries.
- **VS Code Debugging**: Tips to run/debug the nodes inside VS Code.
- **VS Code Extensions (recommended)**: Recommended VS Code extensions for this workspace.
- **Troubleshooting**: Common problems and fixes.

**Project**
- **Description**: Demonstrates ROS 2 publisher/subscriber implemented in Rust using the rclrs (Rust client library) bindings. Use this to learn Rust + ROS 2 integration or as a starting point for your own Rust nodes.

**Prerequisites**
- **ROS 2 Jazzy**: Installed on the host (commonly in `/opt/ros/jazzy`).
- **Rust toolchain**: `rustc`, `cargo` (use `rustup` to install/manage toolchains).
- **colcon**: Used to build the workspace (and the Rust packages wrapped for ROS 2).
- **Development tools**: Recommended — Visual Studio Code, CMake, build-essential packages.

**Quick Setup**
- The repository includes a helper script to prepare the shell environment. From the repository root run:

```bash
bash ./setup_ros2_jazzy.sh
```

- That script will source the system ROS 2 installation and perform any workspace-specific prep steps included in the script.

If you prefer to source ROS manually, run:

```bash
source /opt/ros/jazzy/setup.bash
```

**Build**
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

Notes:
- After a successful `colcon build` the ROS 2 install overlay will be created under `install/` and should be sourced via `source install/setup.bash` before running the nodes.

**Run**
- Run using `ros2 run` (when the package exports the executable):

```bash
# Example (replace names if different in this repo)
ros2 run rust_publisher <executable_name>
ros2 run rust_subscriber <executable_name>
```

- Alternatively run the built executables directly from the install folder:

```bash
ls install/<package>/lib/<package>/
./install/<package>/lib/<package>/<executable>
```

- If you're experimenting, open separate terminals for publisher and subscriber and ensure `source install/setup.bash` has been run in each terminal.

**VS Code Debugging**
- This repository contains a VS Code launch configuration at `/.vscode/launch.json` to help start and attach the debugger to built Rust nodes. To debug effectively:
  - Build the workspace: `colcon build` and `source install/setup.bash`.
  - Open the project in VS Code.
  - Open the Run and Debug view and choose an appropriate configuration (Rust node) from the dropdown.

- Tips for a reliable debug session:
  - Ensure the debug configuration runs the binary from the `install/` tree or sets `cwd` to the package directory so relative paths resolve correctly.
  - Make sure the environment includes ROS 2: either launch VS Code from a shell where `source /opt/ros/jazzy/setup.bash` and `source install/setup.bash` were executed, or add environment variables in `launch.json` (for example set `LD_LIBRARY_PATH` to include `${workspaceFolder}/install/lib`).

**VS Code Extensions (recommended)**
- `Rust Extension Pack` — provides the Rust language support, formatting, and useful tooling for editing and debugging Rust.
- `CMake Tools` — makes editing, configuring and building CMake/CMakeLists-based projects (such as ROS 2 packages) inside VS Code much easier.

**Troubleshooting**
- If `colcon build` fails:
  - Confirm ROS 2 Jazzy is installed and `source /opt/ros/jazzy/setup.bash` was run.
  - Verify Rust toolchain is available (`rustc --version`, `cargo --version`).
- If nodes fail to find message types or libraries at runtime, ensure `source install/setup.bash` is executed in the running shell so the overlayed environment is active.
- If debugger cannot attach or breakpoints are ignored, ensure the binaries were built with debug symbols (default `cargo build` in debug mode) and that the debugger configuration points at the correct executable path.



---
_Enjoy exploring Rust + ROS 2!_
ROS2 Rust demo
