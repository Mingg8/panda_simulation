#include <Eigen/Dense>

#include <ros/ros.h>
#include <vector>
#include <ros/node_handle.h>
#include <ros/package.h>
#include "sensor_msgs/JointState.h"

#include "panda_simulation/NE_matrix.hpp"

using namespace std;
using namespace Eigen;

typedef Matrix<double, 7, 1> V7d;
typedef Matrix<double, 7, 7> M7d;

class Impedance{
  private:
    int argc;
    char **argv;
    
  public:
    Impedance(int argc, char ** argv);
    void startControl();
    Model *model;
    void impedanceControl();
    vector<ros::Publisher> publisher_vec;
    ros::Subscriber robot_state_sub;
    V7d q, dq;
    void JointStateCallback(const sensor_msgs::JointState& msg);
};

