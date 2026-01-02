# 🤖 Robotic Arm ROS 2 Simulation & Control

**Team Deimos – IIT Mandi**

A complete **ROS 2–powered robotic arm project** with **Gazebo (Harmonic) simulation**, **MoveIt 2 motion planning**, and package infrastructure designed for both simulation and real-robot control. 

---

## 📘 1. Project Overview

This repository implements a **6-DOF robotic arm simulation and control stack** using **ROS 2** and **Gazebo**. It provides:

* **URDF model** of the robotic arm
* Different launch files to launch controllers, spawn the robot in Gazebo Harmonic, RViz, etc.
* **MoveIt 2 configuration** for planning and execution
* ROS 2 nodes for controlling arm joints
* Testing demos for motion trajectories

With this workspace, users can **simulate, visualize, plan, and control** a robotic arm entirely in software, with the option to interface with real hardware later.

---

## ⭐ 2. Features

✔ ROS 2 Humble (or Iron) compatible
✔ Gazebo Harmonic simulation support
✔ MoveIt 2 motion planning integration
✔ URDF / Xacro model of robotic arm
✔ RViz visualization configuration
✔ Tested launch workflows from start to finish
✔ Modular ROS 2 packages

---

## 📂 3. Repository Structure

```text
robotic-arm-ros2/
├── my_robotic_arm/                     ← URDF, controllers, nodes
├── my_robotic_arm_moveit_config/       ← MoveIt 2 configuration
├── images-videos/                      ← Media / demos
└── README.md
```

> These are the packages that stay inside the `src` folder of a ROS 2 workspace.
> The user must create (or use) their own workspace to use them.

---

## 🛠 4. Requirements & Supported Software

| Component  | Version / Requirement                               |
| ---------- | --------------------------------------------------- |
| OS         | **Ubuntu 22.04 LTS** (recommended)                  |
| ROS        | **ROS 2 Humble Hawksbill** or **ROS 2 Iron Irwini** |
| Simulator  | **Gazebo Harmonic** (compatible with ROS 2)         |
| Build Tool | `colcon`                                            |
| Python     | 3.10+                                               |
| MoveIt 2   | ROS 2 Motion Planning framework                     |
| C++        | For ROS 2 nodes                                     |

> **NOTE:** Gazebo Harmonic is recommended because compatibility tables show it works reliably with ROS 2 Humble & Iron.

---

## 📦 5. Installation & Workspace Setup

Follow these steps to create a ROS 2 workspace and install all dependencies.

---

### 1️⃣ Setup ROS 2 Environment

Install **ROS 2 Humble or Iron** (if not already installed):

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

---

### 2️⃣ Install Gazebo Harmonic

Gazebo Harmonic is the **latest long-term supported (LTS) Gazebo release**, designed to work cleanly with ROS 2 using `ros_gz` bridges.

---

## ✅ Supported System

| Component | Version          |
| --------- | ---------------- |
| OS        | Ubuntu 22.04 LTS |
| ROS 2     | ROS 2 Humble     |
| Gazebo    | Harmonic         |
| Python    | 3.10+            |

---

## 🧹 Step 0 (IMPORTANT): Remove Old Gazebo Versions (If Any)

```bash
sudo apt remove -y gazebo* ignition*
sudo apt autoremove -y
```

Check that no Gazebo remains:

```bash
gazebo --version
gz sim --version
```

👉 If both commands fail, you are clean.

---

## 🔑 Step 1: Add OSRF Gazebo Repository

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
```

Add the key:

```bash
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.key \
| sudo gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg
```

Add the repository:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable \
$(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

```bash
sudo apt update
```

---

## 📦 Step 2: Install Gazebo Harmonic (Core Simulator)

```bash
sudo apt install -y gz-harmonic
```

Verify:

```bash
gz sim --version
```

Expected:

```text
Gazebo Sim Harmonic X.X.X
```

---

## 🔌 Step 3: Install ROS 2 – Gazebo Harmonic Integration

```bash
sudo apt install -y ros-humble-ros-gz
```

Installed packages include:

* `ros_gz_sim`
* `ros_gz_bridge`
* `ros_gz_image`
* `ros_gz_interfaces`

Verify:

```bash
ros2 pkg list | grep ros_gz
```

---

## 🔁 Step 4: Environment Setup

Add the following to `~/.bashrc`:

```bash
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/usr/share/gz
```

Reload:

```bash
source ~/.bashrc
```

---

## 🧪 Step 5: Test Gazebo Harmonic Standalone

```bash
gz sim empty.sdf
```

If the GUI opens → ✅ Gazebo Harmonic is working.

---

## 🤖 Step 6: Test ROS 2 + Gazebo Harmonic Integration

```bash
ros2 run ros_gz_sim gz_sim
```

Check topics:

```bash
ros2 topic list
```

Expected:

```text
/clock
/joint_states
/tf
```

---

## 🧱 Step 7: Install Gazebo ROS 2 Control (IMPORTANT)

### ✅ Binary Installation (Recommended)

```bash
sudo apt install -y ros-humble-gz-ros2-control
```

Verify:

```bash
ros2 pkg list | grep gz_ros2_control
```

---

### 🛠️ Source Installation (Advanced)

Use source installation only if modification or debugging is required.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/gazebosim/gz_ros2_control.git
git clone https://github.com/ros-controls/ros2_control.git
git clone https://github.com/ros-controls/ros2_controllers.git
```

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 📁 Step 8: Recommended Workspace Variables

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/ros2_ws/install
```

---
## 📁 Step 9: moveit 2 installation

here is the link for moveit2 installation: https://moveit.ai/install-moveit2/binary/

## 📁 6. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Team-Deimos-IIT-Mandi/robotic-arm-ros2.git
cd ~/ros2_ws
```

---

## ⚙️ 7. Build the Workspace

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 8. Running the Simulation (Gazebo)

```bash
ros2 launch my_robotic_arm gazebo_launch.py
```

This will:

* Spawn the robotic arm in Gazebo
* Start controllers
* Publish TF and joint states
* Launch RViz

Test controller:

```bash
ros2 topic pub /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"joint_names: ['joint1','joint2','joint3','Joint_4','Joint_5','Joint_6']
points:
- positions: [0.5, 0.2, -0.3, 0.5, 0.5, 0.5]
  time_from_start: {sec: 2}"
```

---

## 🧠 9. Motion Planning with MoveIt 2

```bash
ros2 launch my_robotic_arm_moveit_config demo.launch.py
```

Use RViz to:

* Set planning scene
* Plan and execute trajectories
* Inspect robot state

---

## 🤖 10. Real Hardware Control via Gazebo Harmonic

```bash
ros2 launch my_robotic_arm gz_moveit.launch.py
```

You should see:

* Gazebo with robot
* RViz with MoveIt tab
* Controllers active

Plan and execute a goal and observe synchronized motion in Gazebo.


---

## ❓ 11. Troubleshooting

**Gazebo not launching / model not visible**
✔ Check URDF and model paths

**MoveIt planning fails**
✔ Verify SRDF and joint limits

**Controllers not responding**
✔ Check controller manager and ROS topics

---

## 🧯 Performance Tips

✔ Use lightweight meshes
✔ Tune controller gains
✔ Use RViz planning plugins efficiently

---

## 📜 License & Credits

This project is released under the **Apache 2.0 License**.

**Credits:**
**Team Deimos – IIT Mandi**

---