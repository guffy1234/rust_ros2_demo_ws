#!/bin/bash

sudo apt update && sudo apt upgrade -y
sudo apt install curl build-essential gcc make -y

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

rustc --version
cargo --version



