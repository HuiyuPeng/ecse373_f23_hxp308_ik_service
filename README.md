# Lab6 Inverse Kinematics Service

## Creating a Service & Client
Services and Clients are another method of passing information between ROS Nodes. Publishers and Subscribers are passive constructs where any number publishers publish a message type on a topic at will and subscribers listen to topics to get whatever information is passed. There can be any number of publishers or subscribers on the same topic.
### Create Nodes and srv
```
catkin_create_pkg ik_service roscpp std_msgs geometry_msgs message_generation ur_kinematics
```
The request will contain a geometry_msgs/Pose, and the response will contain an integer
holding the number of solutions generated and a float vector containing those solutions.
Services and clients use srv types that are analogous to the msg types used in publishers and
subscribers.
The definition of a service srv must be in the srv/ directory of a package. Create and srv/
directory and then create a file named PoseIK.srv in that directory.
```
// Get to the workspace
cd ~/<catkin workspace>
# Create the msg/ directory
mkdir ~/<catin_ws>/srv/ik_service/msgs
# Create the srv/ directory
mkdir ~/<catin_ws>/src/ik_service/srv
# Create an empty files for the JointSolutions and PoseIK service type.
touch src/ik_service/msg/JointSolutions.msg
touch src/ik_service/srv/PoseIK.srv
```
Add the following text to the JointSolutions.msg file.
```
# A vector of joint angles
float64[6] joint_angles
```
This creates the ik_service/JointSolutions message type that will be used in the ik_service/PoseIK.srv definition.
Add the following text to the PoseIK.srv file
```
# Part Pose - This is the request part of the srv
geometry_msgs/Pose part_pose
---
# Number of solutions returned
int8 num_sols
# The joint angles to reach the part
ik_service/JointSolutions[8] joint_solutions
```
The best way to understand the srv definition file and how it will be used is to think of it as defining two message types. The top defines the message type being used for Requests and the bottom is being used to define Responses. The srv definition above defines ik_service::PoseIK::Request, ik_service::PoseIK::Response, and ik_servive::PoseIK which is composed of a request and a response field of types ik_service::PoseIK::Request and ik_service::PoseIK::Response, respectively.
Both the service and client source code must be updated to use the new service type.
```
// Include the definitions of the srv type
#include "ik_service/PoseIK.h"
```
The example service is advertised with the name of add_two_ints. Updated the service name to pose_ik in both the service and client. 
```
/ Service node
ros::ServiceServer service = n.advertiseService("pose_ik", pose_ik);
// Client node
ros::ServiceClient client = n.serviceClient<ik_service::PoseIK>("pose_ik");
```
This should help clarify the inputs to the pose_ik function. Change the types of the inputs to use PoseIK. Remove everything from the pose_ik function in the service code except for the return statement and add a ROS_INFO() call to indicate the service is being called.
```
// Service function
bool pose_ik(ik_service::PoseIK::Request &req,
ik_service::PoseIK::Response &res)
{
ROS_INFO("Pose Information...");
return true;
}
```

In ik_service, remove everything from the pose_ik() function except for the return statement and add a ROS_INFO() call to state that the pose_ik service was called. For now, set the req.num_sols equal to -1 for testing purposes.
nvoke catkin_make and address any errors that may be shown. (It is also good form to eliminate any warning messages thrown during the compile process as well, even if the compile is successful.)
```
// Declare a variable
ik_service::PoseIK ik_pose;
// Declare a Pose variable
geometry_msgs::Pose part_pose;
// Set the variables
part_pose.position.x = 0.5;
ik_pose.request = part_pose;
if (client.call(pose_ik))
{
ROS_INFO("Call to ik_service returned [%i] solutions", pose_ik.res.num_sols);
}
else
{
ROS_ERROR("Failed to call service ik_service");
return 1;
}
```
## Joint Angles from Part Pose
// Include a file from the ur_kinematics package
```
#include "ur_kinematics/ur_kin.h"
```
### The ur_kinematics Package
The ur_kinematics package is part of the ecse_373_ariac ROS package which was downloaded in Part 1. It should already be a declared dependency of the ik_service package (if not, ensure that it becomes one). To begin using it, add a #include statement to the main source code file ur_kin.h header file in the ur_kinematics package.
```
// Include a file from the ur_kinematics package
#include "ur_kinematics/ur_kin.h"
```
This is the header file associated with the src/ur_kin.cpp code file in the ur_kinematics
```
# View the header file (without having to search for it)
gedit `rospack find ur_kinematics`/include/ur_kinematics/ur_kin.h
```
Three functions are provided under the namespace ur_kinematics, and are: forward,
forward_all, and inverse. The inputs and output for each function are indicated with basic
descriptions of how the functions work in the header file.
### Getting Joint Angle Solutions
```
/ Get a forward kinematics solution given joint angles.
// Initial joint angles for the arm
double q[] = {3.14, -1.13, 1.51, 3.77, -1.51, 0};
// Variable to receive forward kinematic solution
double T[4][4];
// Forward kinematic solution
ur_kinematics::forward((double *)&q[0], (double *)&T[0][0]);
// The inputs into the function are technically the memory locations of the memory where the six joint angles are held (q) and the 16 entries for the T matrix can be placed.
```
### The T Matrix
The T Matrix is a transformation matrix in the format covered in class. It describes the position of the end of the stock UR10 arm link reference frame with respect to the arm1_ase_link reference frame. (The very end of a robot that manipulates or “effects” the environment is called an end effector.) 
```
// 2D way to define the same T Matrix
double T[4][4] = {{0.0, -1.0, 0.0, X_POS}, \
                 {0.0, 0.0, 1.0, Y_POS}, \
	         {-1.0, 0.0, 0.0 , Z_POS}, \
	         {0.0, 0.0, 0.0, 1.0}};
```
Use the x, y, & z position from the ik_service req variable as values for X_POS, Y_POS, & Z_POS in the T vector/matrix. Use this T as the input to the ur_kinematics::inverse() function. The output is contained in q_sols. The return value of ur_kinematics::inverse() is the number of solutions contained in q_sols.
```
/ Variable to receive the number of solutions returned
int num_sol;
// Allocate space for up to eight solutions of six joint angles
float q_sols[8][6];
// Inverse kinematic solution(s)
num_sol = ur_kinematics::inverse(&T[0][0], &q_sols[0][0], 0.0);
// The last element is the required precision of the solutions.
```
Use the num_sol and q_sols variables to fill out the ik_service::PoseIK::Response res in the ik_service node. In the ik_client node, add to the ROS_INFO() to show ose
set of joint angles (if there is more than one solution.

