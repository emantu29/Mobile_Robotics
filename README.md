# Mobile Robotics Boat Simulator
In this repository you can download and play a simulator of a boat, equipped with an RGB-D camera, that is able to avoid obstacles and reach the target based on its color.<br/> 
<br/>
The simulator runs on Unity, and it is controlled using ROS2.<br/>
ROS2 and Unity communicate using two packages:<br/>
-ROS-TCP-Connector​<br/>
-ROS-TCP-Endpoint<br/>
<br/>
To run the project, follow these steps:<br/>
1. Download the repository<br/>
2. Open Unity, click on "Add project from disk" and select the folder called "Simulator"<br/>
3. Run the Unity scene<br/>
4. Open the Package Manager from Window -> Package Manager.<br/>
  In the Package Manager window, find and click the + button in the upper lefthand corner of the window. Select "Add package from git URL" and enter the git URL for the desired package.<br/>  
     - For the ROS-TCP-Connector, enter https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector<br/>
     - For the UnitySensors, enter https://github.com/Field-Robotics-Japan/UnitySensors.git?path=/Assets/UnitySensors#v2.0.5<br/>
     - For the UnitySensorsROS, enter https://github.com/Field-Robotics-Japan/UnitySensors.git?path=/Assets/UnitySensorsROS#v2.0.5<br/>
_Note: UnitySensorsROS does not contain UnitySensors._ <br/>
5. Download ROS-TCP-Endpoint from the following link https://github.com/Unity-Technologies/ROS-TCP-Endpoint/releases/tag/ROS2v0.7.0
<br/>
<br/>
The softwares requirements and versions can be found in the file "requirements.txt"


