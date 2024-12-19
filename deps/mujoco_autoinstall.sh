#!/bin/bash

echo "Downloading mujoco to Downloads folder"
cd ~/Downloads
wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz

echo "Extracting archive"
mkdir ~/.mujoco
tar -xvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco/

echo "Exporting lines to .bashrc"
echo -e 'export LD_LIBRARY_PATH=/home/${USER}/.mujoco/mujoco210/bin 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia 
export PATH="$LD_LIBRARY_PATH:$PATH" 
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so' >> ~/.bashrc

echo "Sourcing .bashrc"
source ~/.bashrc

echo "Testing mujoco library"
cd ~/.mujoco/mujoco210/bin
./simulate ../model/humanoid.xml

echo "Done"