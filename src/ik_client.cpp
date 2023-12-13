#include "ik_service/PoseIK.h"
#include "ur_kinematics/ur_kin.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_client");
    ros::ServiceClient client = n.serviceClient<ik_service::PoseIK>("pose_ik");
    ros::NodeHandle n;
    ik_service::PoseIK pose_ik;
    geometry_msgs::Pose part_pose;

    part_pose.position.x = 0.5;
    pose_ik.request.part_pose = part_pose;

    if (client.call(pose_ik))
    {
        ROS_INFO("Calling ik_service gave %d solutions.", pose_ik.response.num_sols);

        for (int i = 0; i < pose_ik.response.num_sols; ++i)
        {
            ROS_INFO("For solution set number %d :", i + 1);
            for (int j = 0; j < 6; ++j)
            {
                ROS_INFO("Joint %d = %f", j + 1, pose_ik.response.joint_solutions[i].joint_angles[j]);
            }
        }
    }

    return 0;
}