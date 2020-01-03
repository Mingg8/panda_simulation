#include "panda_simulation/impedance.hpp"

MatrixXd getJacobian() {

}

void impedance_control(VectorXd q, VectorXd dq) {
    MatrixXd jacobian = getJacobian();
    VectorXd tau = jacobian.transpose() * (-K_im * error - B_im * (jacobian * dq));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "impedance_node");
}