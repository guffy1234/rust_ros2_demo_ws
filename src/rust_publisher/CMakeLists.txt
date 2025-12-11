cmake_minimum_required(VERSION 3.8)
project(rust_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# Build Rust project using Cargo
add_custom_target(${PROJECT_NAME}_cargo ALL
  COMMAND cargo build --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
  # COMMAND cargo build --release --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building Rust publisher with Cargo"
)

# Install the binary
install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/target/debug/${PROJECT_NAME}
  # PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/target/release/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()