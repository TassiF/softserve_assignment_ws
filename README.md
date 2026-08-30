![Isaac Lab](docs/source/_static/isaaclab.jpg)

---

# Pick and place assignment

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![pre-commit](https://img.shields.io/github/actions/workflow/status/isaac-sim/IsaacLab/pre-commit.yaml?logo=pre-commit&logoColor=white&label=pre-commit&color=brightgreen)](https://github.com/isaac-sim/IsaacLab/actions/workflows/pre-commit.yaml)


This framework is based on Isaac Sim/Lab integration with ROS2. A docker container has to be built by using the proprietary NVIDIA docker environment.
A detailed guide on how to do that is provided below.

## Pre-requisites
The framework is entirely containerized. The host only has basic requirements such as:

- Git
- Docker and Docker Compose. For their installation refer to [Docker Installation](https://docs.docker.com/engine/install/ubuntu/)


## How to

### Clone the repo
Clone the workspace directory. It contains the whole Isaac Sim/lab environment and the ROS2 jazzy workspace:
```bash
git clone 
cd softserve_assignment_ws
```
Using the NVIDIA docker framework, we build the containers. The first one ('base') will install Isaac Sim while the second one ('ros2') will provide the ROS2 bridges. I have adjusted the docker compose files in order to build them both by only running:
```bash
sudo python3 docker/container.py start ros2
```
This will pull the images and build the containers, after which it is possible to enter via:
```bash
sudo python3 docker/container.py enter ros2
```
where the working directory is already in the jazzy workspace. 

From here, you can build and run the packages using ROS2. Please follow the guide in the related package (manipulator_sim)[]

## Hardware
Tested using:
- CPU: 16 cores Intel(R) Core(TM) Ultra 7 255H
- GPU: NVIDIA RTX PRO 500 Blackwell Generation Laptop GPU" (6 GiB, sm_120, mempool enabled)


### Documentation

Our [documentation page](https://isaac-sim.github.io/IsaacLab) provides everything you need to get started, including
detailed tutorials and step-by-step guides. Follow these links to learn more about:

- [Installation steps](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html#local-installation)
- [Reinforcement learning](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html)
- [Tutorials](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/index.html)
- [Available environments](https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html)


## IsaacSim/Lab licensing

The Isaac Lab framework is released under [BSD-3 License](LICENSE). The `isaaclab_mimic` extension and its
corresponding standalone scripts are released under [Apache 2.0](LICENSE-mimic). The license files of its
dependencies and assets are present in the [`docs/licenses`](docs/licenses) directory.

Note that Isaac Lab requires Isaac Sim, which includes components under proprietary licensing terms. Please see the [Isaac Sim license](docs/licenses/dependencies/isaacsim-license.txt) for information on Isaac Sim licensing.

Note that the `isaaclab_mimic` extension requires cuRobo, which has proprietary licensing terms that can be found in [`docs/licenses/dependencies/cuRobo-license.txt`](docs/licenses/dependencies/cuRobo-license.txt).