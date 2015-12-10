#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <cstdio>

using namespace std;
using namespace Eigen;

float degree_to_rad(float angle_degree) {
    float angle_rad;
    angle_rad = (angle_degree/360)*2*M_PI;
    return angle_rad;
}

int main()	{
	
	//Point of interest
	Vector3f q_s(12.0, 3.0, 1.0);

	cout << "\nPoint of interest (q): \n" << q_s << endl;

/*********************************ROBOT*********************************/
	//Angle
	float theta = degree_to_rad(28.0);

	//Robot position
	Vector2f p_o;	p_o << 10.8, -2.7;

	//Rotation matrix (robot)
	Matrix2f R_o_b;	R_o_b << cos(theta), -sin(theta),
				sin(theta), cos(theta);

	//Transformation matrix
	Matrix3f T_o_b;

	//Set identity matrix to the transformation matrix
	T_o_b.setIdentity();

	//Set the rotation and translation matrix to the transformation matrix
	T_o_b(0,0) = R_o_b(0,0);	T_o_b(0,1) = R_o_b(0,1);	T_o_b(0,2) = p_o(0);
	T_o_b(1,0) = R_o_b(1,0);	T_o_b(1,1) = R_o_b(1,1);	T_o_b(1,2) = p_o(1);

	cout << "\nTransformation matrix base to origin: \n" << T_o_b << endl;

	
/*********************************SENSOR*********************************/
	//Angle
	float beta = degree_to_rad(41.0);

	//Sensor position
	Vector2f p_b;	p_b << 3.1, 1.2;

	//Rotation matrix (robot)
	Matrix2f R_b_s;	R_b_s << cos(beta), -sin(beta),
				sin(beta), cos(beta);

	//Transformation matrix
	Matrix3f T_b_s;

	//Set identity matrix to the transformation matrix
	T_b_s.setIdentity();

	//Set the rotation and translation matrix to the transformation matrix
	T_b_s(0,0) = R_b_s(0,0);	T_b_s(0,1) = R_b_s(0,1);	T_b_s(0,2) = p_b(0);
	T_b_s(1,0) = R_b_s(1,0);	T_b_s(1,1) = R_b_s(1,1);	T_b_s(1,2) = p_b(1);

	cout << "\nTransformation matrix sensor to base: \n" << T_b_s << endl;


/*******CALCULATING POINT Q WITH RESPECT TO THE BASE FRAME COORDINATE*******/

	//Printing result
	cout << "\nPoint q with respect to the base frame coordinate: \n" << T_b_s * q_s << endl;

/*********CALCULATING POINT Q WITH RESPECT TO THE ORIGIN COORDINATE*********/	

	//Printing result
	cout << "\nPoint q with respect to the origin coordinate: \n" << T_o_b * T_b_s * q_s << endl;


    return 0;
}
