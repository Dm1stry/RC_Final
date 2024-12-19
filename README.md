# Homework on robot control for cource \[F24\] Fundamentals of Robot Control / Основы управления роботами

I have some problems with glfw on my system so i couldn't build plots properly, but in simulation all is fine. On your system plots must be drawn fine

## Dependencies
- conda (i used conda forge)
- MuJoCo
- Simulator (module by Simeon Nedelchev, already placed in deps folder)

### Autoinstallation
Run `deps_autoinstall.sh` and it will automatically install all needed (if nothing have been installed early)

### Manual installation guide for Linux
#### Installing conda-forge (if needed)

``` bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

#### Installation of MuJoCo
> Personally i followed [this](https://gist.github.com/saratrajput/60b1310fe9d9df664f9983b38b50d5da) guide steps, but I'll put it directly here:
- Download the Mujoco library from this [link](https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz).

- Create a hidden folder:
``` bash
mkdir ~/.mujoco
```

- Extract the library to the .mujoco folder:
```bash
tar -xvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco/
```

- Run this to include lines in .bashrc file:
```bash
# Replace user-name with your username
echo -e 'export LD_LIBRARY_PATH=/home/${USER}/.mujoco/mujoco210/bin 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia 
export PATH="$LD_LIBRARY_PATH:$PATH" 
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so' >> ~/.bashrc
```

- Source bashrc:
```bash
source ~/.bashrc
```

- Test that the library is installed.
```bash
cd ~/.mujoco/mujoco210/bin
./simulate ../model/humanoid.xml
```

#### Installing MuJoCo-py
- Run:
```bash
conda create --name mujoco_py python=3.8
conda activate mujoco_py
sudo apt update
sudo apt-get install patchelf
sudo apt-get install python3-dev build-essential libssl-dev libffi-dev libxml2-dev  
sudo apt-get install libxslt1-dev zlib1g-dev libglew1.5 libglew-dev python3-pip

# Clone mujoco-py.
cd ~/.mujoco
git clone https://github.com/openai/mujoco-py
cd mujoco-py
pip3 install -r requirements.txt
pip3 install -r requirements.dev.txt
pip3 install -e . --no-cache
```

##### After installation
- Reboot your machine.
```bash
sudo reboot
```

#### Installing additional packages
```bash
conda activate mujoco_py
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so
# If you get an error like: "ln: failed to create symbolic link '/usr/lib/x86_64-linux-gnu/libGL.so': File exists", it's okay to proceed
pip3 install -U 'mujoco-py<2.2,>=2.1'
```

#### Check installation of mujoco-py
```bash
cd ~/.mujoco/mujoco-py/examples
python3 setting_state.py
```