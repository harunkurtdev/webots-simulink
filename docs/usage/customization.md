The bridge is a crucial component of Webots, responsible for connecting the simulation environment with your custom code. It allows you to interact with the simulation world and retrieve data from it. The bridge provides an interface between the simulation environment and your custom code.

### This will be Changed

## Introduction
In Webots, the bridge plays a vital role in enabling communication between the simulation environment and your custom code. It acts as a conduit for data exchange, allowing you to interact with the simulation world and retrieve information from it. The bridge provides an interface between the simulation environment and your custom code.


![connecting1](../assets/images/usage/customization1.png)

First, we create and define our required variables in Matlab desktop. Then, we add our files to these variables for further usage. These files are taken from Webots GitHub repository and modified to be compatible with Simulink.

The required variables include the robot model, simulation environment settings, and other necessary parameters. After defining these variables, we can use them to interact with the simulation world and retrieve data from it.

![connecting2](../assets/images/usage/customization2.png)

We now proceed to translate the functionality of our custom code into a series of function calls that will bridge the gap between our Matlab desktop environment and the Webots simulation engine.

![connecting3](../assets/images/usage/customization3.png)

In Figure 3, we call the sensor variables and constant variables that we defined in the Webots TextEditor as Constant by Simulink.

![connecting4](../assets/images/usage/customization4.png)

In Figure 4, the values ​​of the fixed variables will be pulled from webots and these values ​​are assigned.

The functions defined in Figure 2 are called by Simulink. Thus, each step is processed by Webots every time the wb_robot_step function is called. It is ensured that it works in common with Simulink.











