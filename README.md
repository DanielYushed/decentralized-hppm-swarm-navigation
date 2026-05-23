# Decentralized Hierarchical HPPM for Multi-Robot Navigation

This repository contains the core architecture, simulation environment, and experimental evaluation scripts for a decentralized hierarchical Homotopy Path Planning Method (HPPM). By integrating an asymmetric Master-Slave hierarchy and a Shared Topological Memory with parametric hysteresis, this framework successfully resolves the mathematical symmetry singularity (Z=0) in multi-robot collisions.

The system has been rigorously tested both in a 2D interactive digital twin environment and through physical laboratory validation using non-holonomic Parallax Scribbler S2 platforms within a 1.8m x 1.8m workspace.

## System Architecture

The project is structured around a centralized visual-servoing system that coordinates decentralized planning and execution.

1. **Vision & Coordination (`Real-Time Swarm Navigation.py`):** Acts as the central node. It uses a camera to track ArUco markers on the robots, predicts their positions using a Kalman filter, and manages the Shared Topological Memory. It asynchronously triggers the mathematical core and sends trajectory updates to the robots via TCP sockets.
2. **Mathematical Core (`HPPM.c`):** The engine of the system. This C program calculates the homotopy paths based on the constraints provided by the central node, outputting the optimal trajectory for each robot.
3. **Communication Node (`Raspberry Code.py`):** Running on a Raspberry Pi 3 aboard each robot. It receives the trajectory data via TCP/IP from the central node, calculates the necessary linear and angular velocities (inverse kinematics), and sends PWM commands to the robot via RS232.
4. **Hardware Execution (`.spin` files):** Code loaded directly onto the Parallax Scribbler S2 microcontrollers to handle low-level motor control based on the received serial commands.

## File Structure

* **`Real-Time Swarm Navigation.py`:** Main Python script for camera capture, ArUco detection, state estimation (Kalman Filter), hierarchical decision-making (Topological Memory), and network transmission.
* **`Raspberry Code.py`:** Python script executed on the Raspberry Pi 3. It acts as a TCP server, calculates the inverse kinematics, and communicates with the Scribbler S2 via serial port.
* **`HPPM.c`:** C source code for the Homotopy Path Planning Method. Compiled into `HPPM.exe` (or equivalent Linux executable), it is called by the main Python script.
* **`Generate_ArUco.py`:** Utility script to generate the ArUco markers used for robot tracking.
* **`.spin` files (`RutaDiff.spin`, `s2.spin`, `FullDuplexSerial.spin`):** Propeller Spin code deployed on the Parallax Scribbler S2 robots for hardware-level execution.

## Requirements

### Central Node (PC)
* Python 3.x
* OpenCV (`opencv-python`, `opencv-contrib-python`)
* NumPy
* GCC (or equivalent C compiler) to build `HPPM.c`

### Robot Node (Raspberry Pi 3)
* Python 3.x
* PySerial (`pyserial`)

### Hardware
* Overhead Camera (calibrated)
* Parallax Scribbler S2 Robots
* Raspberry Pi 3 (one per robot)
* Serial to USB adapters

## Setup & Execution

### 1. Mathematical Core
Compile the C code on the machine running the central node:
```bash
gcc HPPM.c -o HPPM.exe -lm

```

*Ensure the executable path is correctly referenced in the `EJECUTABLE_HPPM` variable within `Real-Time Swarm Navigation.py`.*

### 2. Robot Preparation

1. Load the `.spin` files onto the Parallax Scribbler S2 using the Propeller Tool.
2. Connect the Raspberry Pi to the Scribbler S2 via the serial port.
3. Run the communication script on each Raspberry Pi:
```bash
python3 "Raspberry Code.py"

```



### 3. Central Node Execution

1. Print and attach the generated ArUco markers to the robots.
2. Configure the IP addresses and ports for each robot in the `CONFIG_ROBOTS` dictionary within `Real-Time Swarm Navigation.py`.
3. Execute the main script:
```bash
python "Real-Time Swarm Navigation.py"

```



### Interface Controls

* **Double Left Click:** Set goal for Robot 1.
* **Double Right Click:** Set goal for Robot 2.
* **Double Middle Click:** Set goal for Robot 3.
* **Press 'A':** Toggle Automatic Mode (starts/stops continuous transmission and CSV data recording).
* **Press 'Q':** Quit the application.

## About This Research

This research validates the theoretical advancements in Homotopy Path Planning for multi-agent systems, particularly addressing collision avoidance and deadlock resolution in non-holonomic platforms through a real-time approach.

## Citation

If you find this repository useful for your research, please cite our work:

```bibtex
@article{ruizvera2026hppm,
  title={Experimental validation of a real-time collision avoidance planner based on the improved homotopic continuation method for multi-robot systems},
  author={Ruiz-Vera, Daniel Yushed and D{\'\i}az Arango, Gerardo Ulises and Vazquez-Leal, Hector and Hernandez-Martinez, Luis},
  journal={TBD},
  year={2026}
}

```

## Contact

* **Daniel Yushed Ruiz-Vera** - daniel_yrv@hotmail.com
* **Gerardo Ulises Díaz Arango** - guda.diaz.gd@gmail.com
* Universidad Veracruzana / INAOE

## License

This project is licensed under the MIT License - see the LICENSE file for details.
