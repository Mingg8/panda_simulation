// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: NE_matrix_Franka_w_tool.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 16-Nov-2018 17:26:57
//

// Include Files
#include "panda_simulation/NE_matrix.hpp"


Model::Model() {
	setModel();
}

void Model::setModel() {
	Matrix<double, 3, 3> J_joint_temp;
	J_joint_temp << J_joint_x1, J_joint_xy1, J_joint_xz1,
				J_joint_xy1, J_joint_y1, J_joint_yz1,
				J_joint_xz1, J_joint_yz1, J_joint_z1;
	J_joint.block<3, 3>(0, 0) = J_joint_temp;
	J_joint_temp << J_joint_x2, J_joint_xy2, J_joint_xz2,
				J_joint_xy2, J_joint_y2, J_joint_yz2,
				J_joint_xz2, J_joint_yz2, J_joint_z2;
	J_joint.block<3, 3>(0, 3) = J_joint_temp;
	J_joint_temp << J_joint_x3, J_joint_xy3, J_joint_xz3,
				J_joint_xy3, J_joint_y3, J_joint_yz3,
				J_joint_xz3, J_joint_yz3, J_joint_z3;
	J_joint.block<3, 3>(0, 6) = J_joint_temp;
	J_joint_temp << J_joint_x4, J_joint_xy4, J_joint_xz4,
					J_joint_xy4, J_joint_y4, J_joint_yz4,
					J_joint_xz4, J_joint_yz4, J_joint_z4;
	J_joint.block<3, 3>(0, 9) = J_joint_temp;
	J_joint_temp << J_joint_x5, J_joint_xy5, J_joint_xz5,
					J_joint_xy5, J_joint_y5, J_joint_yz5,
					J_joint_xz5, J_joint_yz5, J_joint_z5;
	J_joint.block<3, 3>(0, 12) = J_joint_temp;
	J_joint_temp << J_joint_x6, J_joint_xy6, J_joint_xz6,
					J_joint_xy6, J_joint_y6, J_joint_yz6,
					J_joint_xz6, J_joint_yz6, J_joint_z6;
	J_joint.block<3, 3>(0, 15) = J_joint_temp;
	J_joint_temp << J_joint_x7, J_joint_xy7, J_joint_xz7,
					J_joint_xy7, J_joint_y7, J_joint_yz7,
					J_joint_xz7, J_joint_yz7, J_joint_z7;
	J_joint.block<3, 3>(0, 18) = J_joint_temp;


	ML << MX1, MX2, MX3, MX4, MX5, MX6, MX7,
		  MY1, MY2, MY3, MY4, MY5, MY6, MY7,
		  MZ1, MZ2, MZ3, MZ4, MZ5, MZ6, MZ7;

	m << m1, m2, m3, m4, m5, m6, m7;

    // DH parameter
	a << 0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0;
	d << 0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0, 0.107;
	alpha << 0.0, -PI_DEFINED / 2.0, PI_DEFINED / 2.0, PI_DEFINED / 2.0,
        -PI_DEFINED / 2.0, PI_DEFINED / 2.0, PI_DEFINED / 2.0, 0.0;
}

MatrixXd Model::getJacobian(V7d q) {
    MatrixXd jacobian(6, 7);
	MatrixXd p_body(3, 7);
	for (int i = 0; i < 7; i++)
	{
        Matrix4d T_tmp = mod_DH_T(a(i), d(i), alpha(i), q(i));
		jacobian.block<3, 1>(3, i) = T_tmp.block<3, 1>(0, 2);
		p_body.block<3, 1>(0, i) = T_tmp.block<3, 1>(0, 3);
	}
	for (int i = 0; i < 7; i++) {
		Vector3d w = jacobian.block<3, 1>(3, i);
        Vector3d r = p_body.block<3, 1>(0, 6) - p_body.block<3, 1>(0, i);
		jacobian.block<3, 1>(0, i) = w.cross(r);
	}
    return jacobian;
}

Matrix3d Model::skew(Vector3d w)
{
	Matrix3d W_temp;
	W_temp << 0.0, -w(2), w(1),
			  w(2), 0.0, -w(0),
			  -w(1), w(0), 0.0;
	return W_temp;
}

Matrix4d Model::SE3_matrix(Matrix3d R, Vector3d p)
{
	Matrix<double, 4, 4> T_tmp;
	T_tmp.setZero();
	T_tmp.block<3, 3>(0, 0) = R;
	T_tmp.block<3, 1>(0, 3) = p;
	T_tmp(3, 3) = 1.0;
	return T_tmp;
}

Matrix3d Model::rot(double rad, char axis)
{
	Matrix3d R_tmp;
	R_tmp.setZero();
	double co, si;
	if (rad == PI_DEFINED / 2.0)
	{
		co = 0.0;
		si = 1.0;
	}
	else if (rad == - PI_DEFINED / 2.0)
	{
		co = 0.0;
		si = -1.0;
	}
	else
	{
		co = cos(rad);
		si = sin(rad);
	}

	if (axis == 'x')
	{
		R_tmp << 1.0, 0.0, 0.0,
			0.0, co, - si,
			0.0, si, co;
	}
	else if (axis == 'y')
	{
		R_tmp << co, 0.0, si,
			0.0, 1.0, 0.0,
			-si, 0.0, co;
	}
	else if (axis == 'z')
	{
		R_tmp << co, -si, 0.0,
			si, co, 0.0,
			0.0, 0.0, 1.0;
	}
	return R_tmp;
}

Matrix4d Model::mod_DH_T(double a, double d, double alpha,
	double theta)
{
	Vector3d p_tmp1, p_tmp2, p_tmp3;
	p_tmp1.setZero();
	p_tmp2.setZero();
	p_tmp3.setZero();

	Matrix3d R_tmp1 = rot(alpha, 'x');
	Matrix3d R_tmp2 = rot(theta, 'z');
	
	p_tmp2(0) = a;
	p_tmp3(2) = d;

	Matrix4d T_tmp1 = SE3_matrix(R_tmp1, p_tmp1);
	Matrix4d T_tmp2 = SE3_matrix(eye3, p_tmp2);
	Matrix4d T_tmp3 = SE3_matrix(R_tmp2, p_tmp1);
	Matrix4d T_tmp4 = SE3_matrix(eye3, p_tmp3);
	Matrix4d T_tmp = T_tmp1 * T_tmp2 * T_tmp3 * T_tmp4;
	return T_tmp;
}

MatrixXd Model::Ad(Matrix<double, 4, 4> T)
{
	Matrix<double, 6, 6> Adjoint_tmp;
	Adjoint_tmp.setZero();

	Matrix3d skew_p;
	skew_p = skew(T.block<3, 1>(0, 3));
	Adjoint_tmp.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
	Adjoint_tmp.block<3, 3>(3, 3) = T.block<3, 3>(0, 0);
	Adjoint_tmp.block<3, 3>(3, 0) = skew_p * T.block<3, 3>(0, 0);
	return Adjoint_tmp;
}

MatrixXd Model::adj(Matrix<double, 6, 1> V)
{
	Matrix<double, 6, 6> adjoint_temp;
	adjoint_temp.setZero();
	
	Matrix3d skew_w = skew(V.block<3, 1>(0, 0));
	Matrix3d skew_v = skew(V.block<3, 1>(3, 0));

	adjoint_temp.block<3, 3>(0, 0) = skew_w;
	adjoint_temp.block<3, 3>(3, 3) = skew_w;
	adjoint_temp.block<3, 3>(3, 0) = skew_v;
	return adjoint_temp;
}

void Model::NE_matrix(V7d q, V7d dq, M7d &M, M7d &C, V7d &Grav)
{
	Matrix<double, 6, 1> Arot;
	Arot << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
	Matrix<double, 42, 7> A;
	A.setZero();
	for (int i = 0; i < 7; i++)
	{
		A.block<6, 1>(6 * i, i) = Arot;
	}

	Matrix<double, 3, 21> R;
	Matrix<double, 4, 4> T_tmp;
	Matrix<double, 3, 7> p_body_pre, p_body_aft;
	Matrix<double, 4, 28> T;
	Matrix<double, 42, 42> Gi;
	R.setZero();
	T.setZero();
	Gi.setZero();

	Matrix3d skew_ML;
	for (int i = 0; i < 7; i++)
	{
		T_tmp = mod_DH_T(a(i), d(i), alpha(i), q(i));

		R.block<3, 3>(0, 3 * i) = T_tmp.block<3, 3>(0, 0);

		p_body_pre.block<3, 1>(0, i) = T_tmp.block<3, 1>(0, 3);
		p_body_aft.block<3, 1>(0, i) = -R.block<3, 3>(0, 3 * i).transpose()
            * p_body_pre.block<3, 1>(0, i);

		T.block<3, 3>(0, 4 * i) = R.block<3, 3>(0, 3 * i).transpose();
		T.block<3, 1>(0, 4 * i + 3) = p_body_aft.block<3, 1>(0, i);
		T(3, 4 * i + 3) = 1.0;

		skew_ML = skew(ML.block<3, 1>(0, i));
		Gi.block<3, 3>(6 * i, 6 * i) = J_joint.block<3, 3>(0, 3 * i);
		Gi.block<3, 3>(6 * i, 6 * i + 3) = skew_ML;
		Gi.block<3, 3>(6 * i + 3, 6 * i) = -skew_ML;
		Gi.block<3, 3>(6 * i + 3, 6 * i + 3) = m(i) * eye3;
	}

    Matrix<double, 6, 6> AdT;
	Matrix<double, 42, 42> W1;
	W1.setZero();
	for (int i = 0; i < 6; i++)
	{
		AdT = Ad(T.block<4, 4>(0, 4 * (i + 1)));
		W1.block<6, 6>(6 * i + 6, 6 * i) = AdT;
	}

	Matrix<double, 42, 42> LL;
	Matrix<double, 4, 4> T_temp;
	LL.setZero();

	for (int i = 0; i < 7; i++)
	{
		for (int j = i; j < 7; j++)
		{
			T_temp = eye4;
			if (j == i)
			{
				LL.block<6, 6>(6 * j, 6 * i) = eye6;
			}
			else
			{
				for (int k = i + 1; k < j + 1; k++)
				{
					T_temp = T.block<4, 4>(0, 4 * k) * T_temp;
				}
				AdT = Ad(T_temp);
				LL.block<6, 6>(6 * j, 6 * i) = AdT;
			}
		}
	}

	Matrix<double, 6, 1> dV0;
	dV0.setZero();
	dV0(5) = gravity;

	Matrix<double, 6, 7> V;
	V.setZero();
	V.block<6, 1>(0, 0) = Arot * dq(0);
	for (int i = 1; i < 7; i++)
	{
		AdT = Ad(T.block<4, 4>(0, 4 * i));
		V.block<6, 1>(0, i) = AdT * V.block<6, 1>(0, i - 1) + Arot * dq(i);
	}

	Matrix<double, 42, 42> ad_V;
	ad_V.setZero();
	Matrix<double, 6, 6> adj_V;
	for (int i = 0; i < 7; i++)
	{
		adj_V = adj(V.block<6, 1>(0, i));
		ad_V.block<6, 6>(6 * i, 6 * i) = adj_V;
	}
	
	Matrix<double, 42, 1> dV_base;
	dV_base.setZero();
	AdT = Ad(T.block<4, 4>(0, 0));
	dV_base.block<6, 1>(0, 0) = AdT * dV0;

	M = A.transpose() * LL.transpose() * Gi * LL * A;
	C = A.transpose() * LL.transpose() * (Gi * LL * ad_V - ad_V.transpose()
        * Gi * LL) * A;
	Grav = A.transpose() * LL.transpose() * Gi * LL * dV_base;
}