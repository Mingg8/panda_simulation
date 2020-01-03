#include "panda_simulation/impedance.hpp"

void impedanceControl(VectorXd q, VectorXd dq) {
    MatrixXd jacobian = model->getJacobian();
    // TODO: get q & dq
    Matrix<double, 7, 7> M, C;
    Matrix<double, 7, 1> Grav;
    NE_matrix(q, dq, *M, *C, *Grav);

    VectorXd tau = jacobian.transpose() * (-K_im * error - B_im * (jacobian * dq));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "impedance_node");
    model = new Model();
    ros::NodeHandle nh;

    panda_arm1_pub = nh.advertise<std_msgs::Float64>("topic_name", 5);
    panda_arm2_pub = nh.advertise<std_msgs::Float64>("topic_name", 5);
    panda_arm3_pub = nh.advertise<std_msgs::Float64>("topic_name", 5);
    panda_arm4_pub = nh.advertise<std_msgs::Float64>("topic_name", 5);
    panda_arm5_pub = nh.advertise<std_msgs::Float64>("topic_name", 5);
    panda_arm6_pub = nh.advertise<std_msgs::Float64>("topic_name", 5);
    panda_arm7_pub = nh.advertise<std_msgs::Float64>("topic_name", 5);

    // TODO: set subscriber
}