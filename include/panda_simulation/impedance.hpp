#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include "panda_simulation/NE_matrix.hpp"

using namespace std;
using namespace Eigen;

Model *model;
ros::Subscriber robot_state_sub;
void robotStateCallback();

ros::Publisher panda_arm1_pub;
ros::Publisher panda_arm2_pub;
ros::Publisher panda_arm3_pub;
ros::Publisher panda_arm4_pub;
ros::Publisher panda_arm5_pub;
ros::Publisher panda_arm6_pub;
ros::Publisher panda_arm7_pub;