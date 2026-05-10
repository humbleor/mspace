# Mspace

[![zread](https://img.shields.io/badge/Ask_Zread-_.svg?style=flat&color=00b0aa&labelColor=000000&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMTYiIGhlaWdodD0iMTYiIHZpZXdCb3g9IjAgMCAxNiAxNiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTQuOTYxNTYgMS42MDAxSDIuMjQxNTZDMS44ODgxIDEuNjAwMSAxLjYwMTU2IDEuODg2NjQgMS42MDE1NiAyLjI0MDFWNC45NjAxQzEuNjAxNTYgNS4zMTM1NiAxLjg4ODEgNS42MDAxIDIuMjQxNTYgNS42MDAxSDQuOTYxNTZDNS4zMTUwMiA1LjYwMDEgNS42MDE1NiA1LjMxMzU2IDUuNjAxNTYgNC45NjAxVjIuMjQwMUM1LjYwMTU2IDEuODg2NjQgNS4zMTUwMiAxLjYwMDEgNC45NjE1NiAxLjYwMDFaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00Ljk2MTU2IDEwLjM5OTlIMi4yNDE1NkMxLjg4ODEgMTAuMzk5OSAxLjYwMTU2IDEwLjY4NjQgMS42MDE1NiAxMS4wMzk5VjEzLjc1OTlDMS42MDE1NiAxNC4xMTM0IDEuODg4MSAxNC4zOTk5IDIuMjQxNTYgMTQuMzk5OUg0Ljk2MTU2QzUuMzE1MDIgMTQuMzk5OSA1LjYwMTU2IDE0LjExMzQgNS42MDE1NiAxMy43NTk5VjExLjAzOTlDNS42MDE1NiAxMC42ODY0IDUuMzE1MDIgMTAuMzk5OSA0Ljk2MTU2IDEwLjM5OTlaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik0xMy43NTg0IDEuNjAwMUgxMS4wMzg0QzEwLjY4NSAxLjYwMDEgMTAuMzk4NCAxLjg4NjY0IDEwLjM5ODQgMi4yNDAxVjQuOTYwMUMxMC4zOTg0IDUuMzEzNTYgMTAuNjg1IDUuNjAwMSAxMS4wMzg0IDUuNjAwMUgxMy43NTg0QzE0LjExMTkgNS42MDAxIDE0LjM5ODQgNS4zMTM1NiAxNC4zOTg0IDQuOTYwMVYyLjI0MDFDMTQuMzk4NCAxLjg4NjY0IDE0LjExMTkgMS42MDAxIDEzLjc1ODQgMS42MDAxWiIgZmlsbD0iI2ZmZiIvPgo8cGF0aCBkPSJNNCAxMkwxMiA0TDQgMTJaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00IDEyTDEyIDQiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIvPgo8L3N2Zz4K&logoColor=ffffff)](https://zread.ai/humbleor/mspace)

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

### 3.1 Simulation

#### Start Simulation
```bash
roslaunch ego_planner 1uav_mid360_sim.launch    # Single UAV (Mid360)
roslaunch ego_planner 1uav_os128_sim.launch     # Single UAV (OS2-128)
roslaunch ego_planner 4uav_mid360_sim.launch    # Swarm (4 UAVs Mid360)
roslaunch drone_detect_lidar 2uav_lidar_detect_sim.launch   # Dual UAV LiDAR detect
```

---

### 3.2 Real World Deployment (实机)

#### Single UAV (单机)

Run the integrated script for Fast-LIO2 and EGO-Planner with Livox support:

```bash
./ego_fastlio2_one_livox.sh
```

#### Swarm (集群)

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
