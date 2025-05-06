# crazyflie_slam
**crazyflie_slam** implements a real-time SLAM algorithm for the Crazyflie nanodrone that relies on the FlowDeck for the state estimate and on the RangerDeck for obstacle information.

## SLAM
SLAM, which stands for Simultaneous Localization and Mapping, refers to a set of algorithms used in robotics and computer vision that enable a device (such as a robot or drone) to create a map of an unknown environment while simultaneously keeping track of its own location within that environment.

For the Crazyflie there were two main questions to answere before starting.
1. What type of sensors, i.e. shall we use the **AI deck** with the camera or **RangerDeck** with 4 Time-of-Flight sensors?
2. Where will the SLAM computations take place? Should we rely solely on the **on-board** processing power, or should we also utilize an external processor, such as a personal computer, for **offloading** tasks?
 
There are two independent choices, resulting in four possible configurations:
- AI deck with on-board processing
- AI deck with off-board processing
- RangerDeck with on-board processing
- RangerDeck with off-board processing :white_check_mark:

We have opted for the last configuration.

We searched for some algorithms that do implement the SLAM task using the available data, i.e. state estimate and four distances.

The main resource used is https://github.com/khazit/CrazySLAM

The structure of our code is the described in the following image.

![architecture](./imgs/architecture.jpg)

As we can see we have a FreeRTOS task added in the Crazyflie firmware that iteratively collect the useful data and then send it over the appchannel to the corresponding endpoint that uses Python-CFlib to read the packet. Furthermore we have the SLAM algorithm that continuously comsumes the received data and updates the model.

## Repository structure
...

## Installation
- download repo
- pip install packets
- Crazyflie FreeRTOS task:
  - download the crazyflie firmware
  - create a new app
  - copy the file in the new app
  - build the firmware
  - flash the new firmware
- Python CFlib
  - configure correctly the radio address
- Run SLAM algorithm