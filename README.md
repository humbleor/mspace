# Mspace

**Mspace** is a ROS Noetic project for UAV autonomy, integrating **Fast-LIO2** for localization and **EGO-Planner** for path planning, with support for **Livox Mid-360** or **Ouster** LiDAR and **Intel RealSense** cameras (for record). This repository supports both single-drone and swarm configurations.

---

## 1. Prerequisites

* **OS:** Ubuntu 20.04
* **ROS:** Noetic
* **PCL:** >= 1.10 (Default in Ubuntu 20.04)
* **Eigen:** >= 3.3.4 (Default in Ubuntu 20.04)
* [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
* [librealsense](https://github.com/IntelRealSense/librealsense)

---

## 2. Build

Clone the repository and compile using the provided script.

> **⚠️ Important:** You must source the `livox_ros_driver2` workspace environment before compiling this project.

```bash
cd YOUR_WORKSPACE/src
git clone https://github.com/humbleor/mspace.git
cd mspace
./compile.sh
```

---

## 3. Usage

### 3.1 Simulation (PX4 + Gazebo)

To run the simulation, you must first set up the PX4 firmware (v1.13.2).

#### Environment Setup

```bash
# Clone PX4 Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git
mv PX4-Autopilot PX4_Firmware
cd PX4_Firmware
# Checkout specific version
git checkout -b tag-1.13.2 v1.13.2
# Update submodules
git submodule update --init --recursive
# Build SITL
make px4_sitl_default gazebo
```

#### Start Simulation



---

### 3.2 Real World Deployment (实机)

#### Single UAV (单机)

Run the integrated script for Fast-LIO2 and EGO-Planner with Livox support:

```bash
./ego_fastlio2_one_livox.sh
```

#### Swarm / Cluster (集群)

For multi-UAV operation, execute the following on the respective machines:

**Host 1 (主机1):**
This script launches MAVROS for the cluster and the planner for UAV 1.

```bash
./ego_fastlio2_swarm_mavros.sh    # Starts MAVROS for hosts
./ego_fastlio2_swarm_uav1.sh      # Starts planning for UAV 1
```

**Host 2 (主机2):**

```bash
./ego_fastlio2_swarm_uav2.sh      # Starts planning for UAV 2
```


### Point Cloud Recording

To record point cloud topics and camera topics for sensors:

```bash
./topiclistOS0.sh
```
