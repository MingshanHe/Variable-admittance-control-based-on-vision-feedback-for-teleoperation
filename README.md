# Variable Admittance Teleoperation

This project is mainly used variable admittance control for teleoperation. In the variable params setting process, I have used RL to make it more flexible.

In this project, using the sim to real theory, training the RL policy in the simulation, and testing the algorithm for the real world.

`control_algorithm`: This folder is mainly for the compliant control framework, like admittance, impedance and hybrid control
`control_strategy`: This folder is mainly for the strategy using the ros control framework.
`robot_controllers`: It have contained some several self-defined ros controllers.
`sensors`: MPU6050 IMU sensors to track the wrist point.

![](Image/1.png)

![](Image/2.png)
