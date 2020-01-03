//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: NE_matrix_Franka_w_tool.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 16-Nov-2018 17:26:57
//
#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>

using namespace Eigen;

#define gravity 9.81
#define PI_DEFINED 3.1415926535897932384626433832795028841971

// constant

typedef const Matrix3d ConstMatrix3;
typedef const Matrix4d ConstMatrix4;
typedef const Eigen::Matrix<double, 6, 6> ConstMatrix6;
typedef Matrix<double, 7, 7> M7d;
typedef Matrix<double, 7, 1> V7d;

ConstMatrix3 eye3 = Matrix3d::Identity();
ConstMatrix4 eye4 = Matrix4d::Identity();
ConstMatrix6 eye6 = Eigen::Matrix<double, 6, 6>::Identity();


class Model {
  public:
	Model();
    void NE_matrix(V7d q, V7d dq, Matrix<double, 7, 7> &M,
        Matrix<double, 7, 7> &C, Matrix<double, 7, 1> &Grav);
    MatrixXd getJacobian(V7d q);
	static Matrix3d skew(Vector3d w);
	static Matrix4d SE3_matrix(Matrix3d R, Vector3d p);
	static Matrix3d rot(double rad, char axis);
	static Matrix4d mod_DH_T(double a, double d, double alpha,
		double theta);
	static MatrixXd Ad(Matrix<double, 4, 4> T);
	static MatrixXd adj(Matrix<double, 6, 1> V);

  private:
	Matrix<double, 3, 21> J_joint;
    Matrix<double, 3, 7> ML;
	Matrix<double, 1, 7> m;
	Matrix<double, 8, 1> a, d, alpha;

    void setModel();

	double m1 = 4.6165;
	double MX1 = 0;
	double MY1 = 0;
	double MZ1 = -0.5777;
	double J_joint_x1 = 1.5938;
	double J_joint_y1 = 1.5938;
	double J_joint_z1 = 0.0212;
	double J_joint_xy1 = 0.1074;
	double J_joint_xz1 = 0.0032;
	double J_joint_yz1 = 0.0014;
	double m2 = 4.4735;
	double MX2 = -0.0057;
	double MY2 = -0.4485;
	double MZ2 = 0;
	double J_joint_x2 = 0.1036;
	double J_joint_y2 = 0.0078;
	double J_joint_z2 = 0.1568;
	double J_joint_xy2 = -0.0053;
	double J_joint_xz2 = 0.0284;
	double J_joint_yz2 = -0.0035;
	double m3 = 2.6049;
	double MX3 = 0.1373;
	double MY3 = 0.0111;
	double MZ3 = -0.275;
	double J_joint_x3 = 0.0874;
	double J_joint_y3 = 0.0895;
	double J_joint_z3 = 0.0135;
	double J_joint_xy3 = 0.0008;
	double J_joint_xz3 = -0.0107;
	double J_joint_yz3 = -0.0048;
	double m4 = 2.5037;
	double MX4 = -0.1461;
	double MY4 = 0.2645;
	double MZ4 = -0.0126;
	double J_joint_x4 = 0.0504;
	double J_joint_y4 = 0.0298;
	double J_joint_z4 = 0.0685;
	double J_joint_xy4 = 0.0304;
	double J_joint_xz4 = 0.0047;
	double J_joint_yz4 = -0.0027;
	double m5 = 1.8632;
	double MX5 = -0.01;
	double MY5 = 0.0484;
	double MZ5 = -0.1427;
	double J_joint_x5 = 0.0426;
	double J_joint_y5 = 0.0368;
	double J_joint_z5 = 0.0058;
	double J_joint_xy5 = -0.0037;
	double J_joint_xz5 = -0.0061;
	double J_joint_yz5 = 0.0077;
	double m6 = 1.3229;
	double MX6 = 0.0803;
	double MY6 = -0.0089;
	double MZ6 = -0.0287;
	double J_joint_x6 = 0.0066;
	double J_joint_y6 = 0.0119;
	double J_joint_z6 = 0.0112;
	double J_joint_xy6 = 0.0002;
	double J_joint_xz6 = 0;
	double J_joint_yz6 = 0.0008;
	double m7 = 0.9699;
	double MX7 = 0.0062;
	double MY7 = -0.0005;
	double MZ7 = 0.0588;
	double J_joint_x7 = 0.0049;
	double J_joint_y7 = 0.0063;
	double J_joint_z7 = 0.0018;
	double J_joint_xy7 = 0.0012;
	double J_joint_xz7 = 0.0017;
	double J_joint_yz7 = -0.0006;
};
