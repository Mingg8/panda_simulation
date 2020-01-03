#include "panda_simulation/impedance.hpp"
#include <std_msgs/Float64.h>

// TODO: set error

Impedance::Impedance(int _argc, char ** _argv)
    : argc(_argc), argv(_argv) {};

void Impedance::impedanceControl() {
    V7d error;
    error.setZero();

    MatrixXd jacobian = model->getJacobian(q);
    M7d K_im, B_im;
    Matrix<double, 7, 7> M, C;
    Matrix<double, 7, 1> Grav;

    K_im.setZero();
    B_im.setZero();
    K_im.diagonal() << 1500, 1500, 1500, 65, 65, 65;
    B_im.diagonal() << 100, 100, 100, 20, 20, 20;

    model->NE_matrix(q, dq, M, C, Grav);

    VectorXd tau = Grav + C * dq + 
        jacobian.transpose() * (-K_im * error - B_im * (jacobian * dq));
    tau = Grav;

    std_msgs::Float64 msg;
    for (int i = 0; i < 7; i++) {
        msg.data = tau(i);
        (publisher_vec[i]).publish(msg);
    }
}

void Impedance::JointStateCallback(const sensor_msgs::JointState& msg) {
    for (int i = 0; i < 7; i ++) {
        q(i) = msg.position[i];
        dq(i) = msg.velocity[i];
    }
    cout << q.transpose() << endl;
    cout << dq.transpose() << endl;
}

void Impedance::startControl() {
    ros::init(argc, argv, "impedance_node");
    ros::NodeHandle nh;

    string topic_name1 = "/panda_arm";
    string topic_name2 = "_controller/command";

    for (int i = 0; i < 7; i++) {
        string msg = topic_name1 + to_string(i) + topic_name2;
        ros::Publisher pub = nh.advertise<std_msgs::Float64>(msg.c_str(), 5);
        publisher_vec.push_back(pub);
    }

    robot_state_sub = nh.subscribe("/joint_states", 100,
        &Impedance::JointStateCallback, this);

    ros::Rate loop_rate(10);
    model = new Model();

    while (ros::ok()) {
        impedanceControl();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    Impedance *impedance = new Impedance(argc, argv);
    impedance->startControl();
}