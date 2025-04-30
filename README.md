# Balancing a Segway

A Reinforcement Learning project for controlling a Segway. Developed for RO3002: Reinforcement Learning course.

## Project Overview

This repository contains the software and configuration for a physical cart-pendulum (Segway) system that uses reinforcement learning techniques for balance control. The project integrates hardware components with RL algorithms to achieve stable operation.

Please refer to final_report.pdf for Abstract, Related Work, Problem Statement, RL Formulation, Methodology and Results. All multimedia can be found embedded within the multimedia/ folder. 
## Repository Structure

- **hardware/** - Hardware interface and control components
  - **deadband_compensation/** - Tools for motor deadband calibration and compensation
  - **encoder_characterisation/** - Code for characterizing and calibrating encoders
  - **main/** - Main control system implementation
    - **controller.py** - Traditional control implementation
    - **controller_rl.py** - Reinforcement learning-based controller
  - **robot_logs/** - System operation logs
  - **tests/** - IMU Calibration
  - **tuning/** - Hardware RL Agent Fine-Tuning

- **python/** - Python-based components
  - **environment/** - Simulation environment for RL training
  - **models/** - RL model implementations

- **Training/** - Training resources
  - **Saved Models/** - Trained RL model checkpoints

## Setup and Installation

1. Clone this repository
2. Install the required dependencies:
    ``` pip install -r requirements.txt ```

## Hardware Components

The system uses:
- Rhino 500 RPM 6kgcm DC Motors with a 2262 PPR Quadrature Encoder
- Cytron MDD10A Motor Driver
- Raspberry Pi 3B+ Microcontroller
- MPU 6050 IMU
- Orange 2200 mAh 3S LiPo Battery

## Usage

To run the RL controller, run the following command under hardware/main
``` 
sudo python3 main.py
```

**Note:** Modify line 111 and 114 in main.py to change RL agent being used.

To run the PID baseline controller, run the following command under hardware/main
```
sudo python3 pid_main.py
```

### Hardware Calibration

To make the deadband compensation measurements run the following command under hardware/deadband_compensation:

```
sudo python3 main.py
```

To find the deadband coefficients, and plots run the following command under hardware/deadband_compensation:

```
sudo python3 find_constants_10.py
```

For encoder characterisation, run the following command under hardware/encoder_characterisation:
```
sudo python3 main.py
```

To compare different filters over the numerical derivative, run the following command under hardware/encoder_characterisation:
```
sudo python3 filters.py
```

To calibrate the IMU and find offsets, run the following command under hardware/tests:
```
sudo python3 imu_test.py
```

### Hardware Fine-Tuning
Select pre-trained model to be further fine tuned, and modify line 11 and 20 on continue_hardware_training.py. Then run the following command under hardware/tuning:

```
sudo python3 continue_hardware_training.py
```
