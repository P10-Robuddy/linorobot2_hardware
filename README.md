# Polybot linorobot2_hardware

## Installation

All software mentioned in this guide must be installed on a Polybot with a Raspberry Pi 4 running Ubuntu Server 22.04.3 and a Teensy.

### 1. ROS2 and linorobot2 installation

It is assumed that you already have ROS2 and linorobot2 package installed. If you haven't, go to [linorobot2](https://github.com/P9-Robuddy/linorobot2) package for installation guide.

### 2. Download linorobot2_hardware

    cd $HOME
    git clone https://github.com/p9-robuddy/linorobot2_hardware

### 3. Install dependencies

    sudo apt install -y python3.10-venv
    sudo apt install -y libusb-dev
    sudo apt install -y screen

### 4. Install PlatformIO

Download and install platformio. [Platformio](https://platformio.org/) allows you to develop, configure, and upload the firmware without the Arduino IDE. This means that you can upload the firmware remotely which is ideal on headless setup especially when all components have already been fixed.

    curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 get-platformio.py

Add platformio to your PATH:

    echo "export PATH=\"$HOME/.platformio/penv/bin:$PATH\"" >> $HOME/.bashrc
    source $HOME/.bashrc

### 5. UDEV Rule

Download the udev rules from Teensy's website:

    wget https://www.pjrc.com/teensy/00-teensy.rules

and copy the file to /etc/udev/rules.d :

    sudo cp 00-teensy.rules /etc/udev/rules.d/

### 6. Namespace

You should set a namespace in _config.h_. Here it is set to polybot01.

    #define ROS_NAMESPACE "polybot01"

## Calibration

Before proceeding, **ensure that your robot is elevated and the wheels aren't touching the ground**.

You might want to source _bashrc_ again:

    source $HOME/.bashrc

### 1. Motor Check

Go to calibration folder and upload the firmware:

    cd linorobot2_hardware/calibration
    pio run --target upload -e polybot

Start spinning the motors by running:

    screen /dev/ttyACM0

On the terminal type `spin` and press the enter key.

The wheels will spin one by one for 10 seconds from Motor1 to Motor4. Check if each wheel's direction is spinning **forward** and take note of the motors that are spinning in the opposite direction. Set MOTORX_INV constant in _lino_base_config.h_ to `true` to invert the motor's direction. Reupload the calibration firmware once you're done. Press `Ctrl` + `a` + `d` to exit the screen terminal.

    cd linorobot2_hardware/calibration
    pio run --target upload -e polybot

### 2. Encoder Check

Open your terminal and run:

    screen /dev/ttyACM0

Type `sample` and press the enter key. Verify if all the wheels are spinning **forward**. Redo the previous step if there are motors still spinning in the opposite direction.

You'll see a summary of the total encoder readings and counts per revolution after the motors have been sampled. If you see any negative number in the MOTOR ENCODER READINGS section, invert the encoder pin by setting `MOTORX_ENCODER_INV` in _lino_base_config.h_ to `true`. Reupload the calibration firmware to check if the encoder pins have been reconfigured properly:

    cd linorobot2_hardware/calibration
    pio run --target upload -e polybot
    screen /dev/ttyACM0

Type `sample` and press the enter key. Verify if all encoder values are now **positive**. Redo this step if you missed out any.

### 3. Counts Per Revolution

On the previous instruction where you check the encoder reads for each motor, you'll see that there's also COUNTS PER REVOLUTION values printed on the screen. If you have defined `MOTOR_OPERATING_VOLTAGE` and `MOTOR_POWER_MEASURED_VOLTAGE`, you can assign these values to `COUNTS_PER_REVX` constants in _lino_base_config.h_ to have a more accurate model of the encoder.

## Upload the firmware

Ensure that the robot pass all the requirements before uploading the firmware:

- Defined the correct motor rpm.
- Motors' IDs are correct.
- Motors spin in the right direction.
- Encoders' signs are correct.
- Defined the correct encoder's COUNTS_PER_REV.
- Defined the correct robot type.
- Defined the correct motor driver.
- Defined the correct IMU.
- Defined the correct wheel diameter.
- Defined the correct distance between wheels.

Run:

    cd linorobot2_hardware/firmware
    pio run --target upload -e polybot

## Testing the robot

You might want to source the install setup again.

    cd $HOME/linorobot_ws
    source install/setup.bash

Also make sure to run the bringup script first from [here](https://github.com/P9-Robuddy/linorobot2).

### 1. Run the micro-ROS agent

This will allow the robot to receive Twist messages to control the robot, and publish odometry and IMU data straight from the microcontroller. Compared to Linorobot's ROS1 version, the odometry and IMU data published from the microcontroller use standard ROS2 messages and do not require any relay nodes to reconstruct the data to complete [sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) and [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages.

Run the agent:

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

### 2. Drive around

Run teleop_twist_keyboard package and follow the instructions on the terminal on how to drive the robot:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard 

### 3. Check the topics

Check if the odom and IMU data are published:

    ros2 topic list

Now you should see the following topics:

    /cmd_vel
    /imu/data
    /odom/unfiltered
    /parameter_events
    /rosout

Echo odometry data:

    ros2 topic echo /odom/unfiltered

Echo IMU data:

    ros2 topic echo /imu/data

## Troubleshooting Guide

### 1. One of my motor isn't spinning

- Check if the motors are powered.
- Check if you have bad wiring.
- Check if you have misconfigured the motor's pin assignment in lino_base_config.h.
- Check if you uncommented the correct motor driver (ie. `USE_GENERIC_2_IN_MOTOR_DRIVER`)
- Check if you assigned the motor driver pins under the correct motor driver constant. For instance, if you uncommented `USE_GENERIC_2_IN_MOTOR_DRIVER`, all the pins you assigned must be inside the `ifdef USE_GENERIC_2_IN_MOTOR_DRIVER` macro.

### 2. Wrong wheel is spinning during calibration process

- Check if the motor drivers have been connected to the correct microcontroller pin.
- Check if you have misconfigured the motor's pin assignment in lino_base_config.h.

### 3 One of my encoders has no reading (0 value)

- Check if the encoders are powered.
- Check if you have bad wiring.
- Check if you have misconfigured the encoder's pin assignment in lino_base_config.h.

### 4. The wheels only spin in one direction

- Check if the Teensy's GND pin is connected to the motor driver's GND pin.

### 5. The motor doesn't change it's direction after setting the INV to true

- Check if the Teensy's GND pin is connected to the motor driver's GND pin.

### 6. Nothing's printing when I run the screen app

- Check if you're passing the correct serial port. Run:

        ls /dev/ttyACM*

    and ensure that the available serial port matches the port you're passing to the screen app.

- Check if you forgot to copy the udev rule:

        ls /etc/udev/rules.d/00-teensy.rules 

    Remember to restart your computer if you just copied the udev rule.

### 7. The firmware was uploaded but nothing's happening

- Check if you're assigning the correct Teensy board when uploading the firmware. If you're unsure which Teensy board you're using, take a look at the label on the biggest chip found in your Teensy board and compare it with the boards shown on PJRC's [website](https://www.pjrc.com/teensy/).

### 8. The robot's forward motion is not straight

- This happens when the target velocity is more than or equal the motor's RPM (usually happens on low RPM motors). To fix this, set the `MAX_RPM_RATIO` lower to allow the PID controller to compensate for errors.

### 9. The robot rotates after braking

- This happens due to the same reason as 7. When the motor hits its maximum rpm and fails to reach the target velocity, the PID controller's error continously increases. The abrupt turning motion is due to the PID controller's attempt to further compensate the accumulated error. To fix this, set the `MAX_RPM_RATIO` lower to allow the PID controller to compensate for errors while moving to avoid huge accumulative errors when the robot stops.
