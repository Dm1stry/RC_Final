#!/bin/bash

echo "Installing mujoco-py"

conda create --name mujoco_py python=3.8
conda activate mujoco_py
sudo apt update
sudo apt-get install patchelf
sudo apt-get install python3-dev build-essential libssl-dev libffi-dev libxml2-dev
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
sudo apt-get install libxslt1-dev zlib1g-dev libglew1.5 libglew-dev python3-pip
sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so

# Clone mujoco-py.
cd ~/.mujoco
git clone https://github.com/openai/mujoco-py
cd mujoco-py
pip3 install -r requirements.txt
pip3 install -r requirements.dev.txt
pip3 install -e . --no-cache
pip3 install -U 'mujoco-py<2.2,>=2.1'

echo "Done"
 
echo "You have to reboot your system"
