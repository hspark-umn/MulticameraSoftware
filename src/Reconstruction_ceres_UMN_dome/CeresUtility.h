#ifndef CERESUTILITY_H
#define CERESUTILITY_H
#include "MathUtility.h"
//#include "Classifier.h"
#include "DataUtility.h"
#include "epnp.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace std;
#define POINT_AT_INFINITY_ZERO 1e-2
#define PI 3.14159265

using ceres::Problem;
using ceres::AutoDiffCostFunction;

class BALProblem {
 public:
  ~BALProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observations()       const { return num_observations_;               }
  const double* observations() const { return observations_;                   }
  double* mutable_cameras()          { return parameters_;                     }
  double* mutable_points()           { return parameters_  + 7 * num_cameras_; }

  double* mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 7;
  }
  double* mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
  }


  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
};
void Reprojection(double *camera, double *point, double *add, double *measurement, double *res);
void Reprojection1(double *camera, double *point, double *add, double *measurement, double *res);
void CeresSolverUMNDome(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> &vK, vector<CvMat *> &cP, CvMat *X, vector<Camera> vCamera);
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {

  SnavelyReprojectionError(double observed_x, double observed_y, double omega, double princ_x1, double princ_y1, double K11, double K22, double K13, double K23)
      : observed_x(observed_x), observed_y(observed_y), omega_(omega), 
		princ_x1_(princ_x1), princ_y1_(princ_y1), K11_(K11), K22_(K22), K13_(K13), K23_(K23) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {

	// Set intrinsic parameter
	T omega = T(omega_);
	T tan_omega_half_2 = T(2)*tan(T(omega_/2));
	T princ_x1 = T(princ_x1_);
	T princ_y1 = T(princ_y1_);
	T K11 = T(K11_);
	T K22 = T(K22_);
	T K13 = T(K13_);
	T K23 = T(K23_);

	T K12 = T(0);
	T K21 = T(0);
	T K31 = T(0);
	T K32 = T(0);
	T K33 = T(1);

	// Set orientation
	T qw = camera[0];
	T qx = camera[1];
	T qy = camera[2];
	T qz = camera[3];
	T q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
	qw /= q_norm;
	qx /= q_norm;
	qy /= q_norm;
	qz /= q_norm;

	T R11 = T(1.0)-T(2)*qy*qy-T(2)*qz*qz; 		T R12 = T(2)*qx*qy-T(2)*qz*qw;			T R13 = T(2)*qx*qz+T(2)*qy*qw;
	T R21 = T(2)*qx*qy+T(2)*qz*qw;				T R22 = T(1.0)-T(2)*qx*qx-T(2)*qz*qz;	T R23 = T(2)*qz*qy-T(2)*qx*qw;
	T R31 = T(2)*qx*qz-T(2)*qy*qw;				T R32 = T(2)*qy*qz+T(2)*qx*qw;			T R33 = T(1.0)-T(2)*qx*qx-T(2)*qy*qy;


	// Set translation
	T C1 = camera[4];
	T C2 = camera[5];
	T C3 = camera[6];

	T X1 = point[0];
	T X2 = point[1];
	T X3 = point[2];


	
	//cout << observed_x << " " << observed_x << endl;
	//cout << X1 << " " << X2 << " " << X3 << endl;

	//cout << R11 << " " << R12 << " " << R13 << endl;
	//cout << R21 << " " << R22 << " " << R23 << endl;
	//cout << R31 << " " << R32 << " " << R33 << endl;

	// Building projection 
	T RX1 = R11*X1+R12*X2+R13*X3;
	T RX2 = R21*X1+R22*X2+R23*X3;
	T RX3 = R31*X1+R32*X2+R33*X3;

	T KRX1 = K11*RX1+K12*RX2+K13*RX3;
	T KRX2 = K21*RX1+K22*RX2+K23*RX3;
	T KRX3 = K31*RX1+K32*RX2+K33*RX3;

	T RC1 = R11*C1+R12*C2+R13*C3;
	T RC2 = R21*C1+R22*C2+R23*C3;
	T RC3 = R31*C1+R32*C2+R33*C3;

	T KRC1 = K11*RC1+K12*RC2+K13*RC3;
	T KRC2 = K21*RC1+K22*RC2+K23*RC3;
	T KRC3 = K31*RC1+K32*RC2+K33*RC3;

	T proj1 = KRX1-KRC1;
	T proj2 = KRX2-KRC2;
	T proj3 = KRX3-KRC3;

	T u = proj1/proj3;
	T v = proj2/proj3;

	T u_n = u - princ_x1;
	T v_n = v - princ_y1;

	T r_u = sqrt(u_n*u_n+v_n*v_n);
	T r_d = T(1)/omega*atan(r_u*tan_omega_half_2);

	T u_d_n = r_d/r_u * u_n;
	T v_d_n = r_d/r_u * v_n;

	T u_d = u_d_n + princ_x1;
	T v_d = v_d_n + princ_y1;

    // The error is the difference between the predicted and observed position.
    residuals[0] = u_d - T(observed_x);
    residuals[1] = v_d - T(observed_y);

	//printf("%lf %lf\n", u_d, v_d);
	//printf("%lf %lf\n", residuals[0], residuals[1]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
									 const double omega,
									 const double princ_x1,
									 const double princ_y1,
									 const double K11,
									 const double K22,
									 const double K13,
									 const double K23) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 7, 3>(
                new SnavelyReprojectionError(observed_x, observed_y, omega, princ_x1, princ_y1, K11, K22, K13, K23)));
  }

  double observed_x;
  double observed_y;
  double omega_;
  double princ_x1_;
  double princ_y1_;
  double K11_;
  double K22_;
  double K13_;
  double K23_;


};
void GetParameterForSBA_Distortion(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> cP, CvMat *X, vector<CvMat *> vK, vector<int> visibleStructureID, vector<int> &vCameraIdx, BALProblem &bal_problem);

struct SnavelyReprojectionError1 {

  SnavelyReprojectionError1(double observed_x, double observed_y, double omega, double K11, double K22, double K13, double K23)
      : observed_x(observed_x), observed_y(observed_y), omega_(omega), 
		K11_(K11), K22_(K22), K13_(K13), K23_(K23) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {

	// Set intrinsic parameter
	T omega = T(omega_);
	T tan_omega_half_2 = T(2)*tan(T(omega_/2));
	T K11 = T(K11_);
	T K22 = T(K22_);
	T K13 = T(K13_);
	T K23 = T(K23_);

	T K12 = T(0);
	T K21 = T(0);
	T K31 = T(0);
	T K32 = T(0);
	T K33 = T(1);

	// Set orientation
	T qw = camera[0];
	T qx = camera[1];
	T qy = camera[2];
	T qz = camera[3];
	T q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
	qw /= q_norm;
	qx /= q_norm;
	qy /= q_norm;
	qz /= q_norm;

	T R11 = T(1.0)-T(2)*qy*qy-T(2)*qz*qz; 		T R12 = T(2)*qx*qy-T(2)*qz*qw;			T R13 = T(2)*qx*qz+T(2)*qy*qw;
	T R21 = T(2)*qx*qy+T(2)*qz*qw;				T R22 = T(1.0)-T(2)*qx*qx-T(2)*qz*qz;	T R23 = T(2)*qz*qy-T(2)*qx*qw;
	T R31 = T(2)*qx*qz-T(2)*qy*qw;				T R32 = T(2)*qy*qz+T(2)*qx*qw;			T R33 = T(1.0)-T(2)*qx*qx-T(2)*qy*qy;


	// Set translation
	T C1 = camera[4];
	T C2 = camera[5];
	T C3 = camera[6];

	T X1 = point[0];
	T X2 = point[1];
	T X3 = point[2];


	
	//cout << observed_x << " " << observed_x << endl;
	//cout << X1 << " " << X2 << " " << X3 << endl;

	//cout << R11 << " " << R12 << " " << R13 << endl;
	//cout << R21 << " " << R22 << " " << R23 << endl;
	//cout << R31 << " " << R32 << " " << R33 << endl;

	// Building projection 
	T RX1 = R11*X1+R12*X2+R13*X3;
	T RX2 = R21*X1+R22*X2+R23*X3;
	T RX3 = R31*X1+R32*X2+R33*X3;

	T KRX1 = K11*RX1+K12*RX2+K13*RX3;
	T KRX2 = K21*RX1+K22*RX2+K23*RX3;
	T KRX3 = K31*RX1+K32*RX2+K33*RX3;

	T RC1 = R11*C1+R12*C2+R13*C3;
	T RC2 = R21*C1+R22*C2+R23*C3;
	T RC3 = R31*C1+R32*C2+R33*C3;

	T KRC1 = K11*RC1+K12*RC2+K13*RC3;
	T KRC2 = K21*RC1+K22*RC2+K23*RC3;
	T KRC3 = K31*RC1+K32*RC2+K33*RC3;

	T proj1 = KRX1-KRC1;
	T proj2 = KRX2-KRC2;
	T proj3 = KRX3-KRC3;

	T u = proj1/proj3;
	T v = proj2/proj3;

	T u_n = (u - K13)/K11;
	T v_n = (v - K23)/K22;

	T r_u = sqrt(u_n*u_n+v_n*v_n);
	T r_d = T(1)/omega*atan(r_u*tan_omega_half_2);

	T u_d_n = r_d/r_u * u_n;
	T v_d_n = r_d/r_u * v_n;

	T u_d = K11*u_d_n + K13;
	T v_d = K22*v_d_n + K23;

    // The error is the difference between the predicted and observed position.
    residuals[0] = u_d - T(observed_x);
    residuals[1] = v_d - T(observed_y);

	//printf("%lf %lf\n", u_d, v_d);
	//printf("%lf %lf\n", residuals[0], residuals[1]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
									 const double omega,
									 const double K11,
									 const double K22,
									 const double K13,
									 const double K23) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError1, 2, 7, 3>(
                new SnavelyReprojectionError1(observed_x, observed_y, omega, K11, K22, K13, K23)));
  }

  double observed_x;
  double observed_y;
  double omega_;
  double K11_;
  double K22_;
  double K13_;
  double K23_;


};

struct SnavelyReprojectionError3 {

  SnavelyReprojectionError3(double observed_x, double observed_y, double omega, double princ_x1, double princ_y1, double K11, double K22, double K13, double K23)
      : observed_x(observed_x), observed_y(observed_y), omega_(omega), princ_x1_(princ_x1), princ_y1_(princ_y1),
		K11_(K11), K22_(K22), K13_(K13), K23_(K23) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {

	// Set intrinsic parameter
	T omega = T(omega_);
	T tan_omega_half_2 = T(2)*tan(T(omega_/2));
	T princ_x1 = T(princ_x1_);
	T princ_y1 = T(princ_y1_);
	T K11 = T(K11_);
	T K22 = T(K22_);
	T K13 = T(K13_);
	T K23 = T(K23_);

	T K12 = T(0);
	T K21 = T(0);
	T K31 = T(0);
	T K32 = T(0);
	T K33 = T(1);

	// Set orientation
	T qw = camera[0];
	T qx = camera[1];
	T qy = camera[2];
	T qz = camera[3];
	T q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
	qw /= q_norm;
	qx /= q_norm;
	qy /= q_norm;
	qz /= q_norm;

	T R11 = T(1.0)-T(2)*qy*qy-T(2)*qz*qz; 		T R12 = T(2)*qx*qy-T(2)*qz*qw;			T R13 = T(2)*qx*qz+T(2)*qy*qw;
	T R21 = T(2)*qx*qy+T(2)*qz*qw;				T R22 = T(1.0)-T(2)*qx*qx-T(2)*qz*qz;	T R23 = T(2)*qz*qy-T(2)*qx*qw;
	T R31 = T(2)*qx*qz-T(2)*qy*qw;				T R32 = T(2)*qy*qz+T(2)*qx*qw;			T R33 = T(1.0)-T(2)*qx*qx-T(2)*qy*qy;


	// Set translation
	T C1 = camera[4];
	T C2 = camera[5];
	T C3 = camera[6];

	T X1 = point[0];
	T X2 = point[1];
	T X3 = point[2];


	
	//cout << observed_x << " " << observed_x << endl;
	//cout << X1 << " " << X2 << " " << X3 << endl;

	//cout << R11 << " " << R12 << " " << R13 << endl;
	//cout << R21 << " " << R22 << " " << R23 << endl;
	//cout << R31 << " " << R32 << " " << R33 << endl;

	// Building projection 
	T RX1 = R11*X1+R12*X2+R13*X3;
	T RX2 = R21*X1+R22*X2+R23*X3;
	T RX3 = R31*X1+R32*X2+R33*X3;

	T KRX1 = K11*RX1+K12*RX2+K13*RX3;
	T KRX2 = K21*RX1+K22*RX2+K23*RX3;
	T KRX3 = K31*RX1+K32*RX2+K33*RX3;

	T RC1 = R11*C1+R12*C2+R13*C3;
	T RC2 = R21*C1+R22*C2+R23*C3;
	T RC3 = R31*C1+R32*C2+R33*C3;

	T KRC1 = K11*RC1+K12*RC2+K13*RC3;
	T KRC2 = K21*RC1+K22*RC2+K23*RC3;
	T KRC3 = K31*RC1+K32*RC2+K33*RC3;

	T proj1 = KRX1-KRC1;
	T proj2 = KRX2-KRC2;
	T proj3 = KRX3-KRC3;

	T u = proj1/proj3;
	T v = proj2/proj3;

	T u_n = u - princ_x1;
	T v_n = v - princ_y1;

	T r_u = sqrt(u_n*u_n+v_n*v_n);
	T r_d = T(1)/omega*atan(r_u*tan_omega_half_2);

	T u_d_n = r_d/r_u * u_n;
	T v_d_n = r_d/r_u * v_n;

	T u_d = u_d_n + princ_x1;
	T v_d = v_d_n + princ_y1;

    // The error is the difference between the predicted and observed position.
    residuals[0] = u_d - T(observed_x);
    residuals[1] = v_d - T(observed_y);

	//printf("%lf %lf\n", u_d, v_d);
	//printf("%lf %lf\n", observed_x, observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
									 const double omega,
									 const double princ_x1,
									 const double princ_y1,
									 const double K11,
									 const double K22,
									 const double K13,
									 const double K23) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError3, 2, 7, 3>(
                new SnavelyReprojectionError3(observed_x, observed_y, omega, princ_x1, princ_y1, K11, K22, K13, K23)));
  }

  double observed_x;
  double observed_y;
  double omega_;
  double K11_;
  double K22_;
  double K13_;
  double K23_;
  double princ_x1_;
  double princ_y1_;

};

void Reprojection(double *camera, double *point, double K11, double K22, double K13, double K23, double *measurement, double *res);

struct SnavelyReprojectionError4 {

	SnavelyReprojectionError4(double observed_x, double observed_y, double K11, double K22, double K13, double K23)
	: observed_x(observed_x), observed_y(observed_y), K11_(K11), K22_(K22), K13_(K13), K23_(K23) {}

	template <typename T>
	bool operator()(const T* const camera,
		const T* const point,
		T* residuals) const {

		// Set intrinsic parameter
		T K11 = T(K11_);
		T K22 = T(K22_);
		T K13 = T(K13_);
		T K23 = T(K23_);

		T K12 = T(0);
		T K21 = T(0);
		T K31 = T(0);
		T K32 = T(0);
		T K33 = T(1);

		// Set orientation
		T qw = camera[0];
		T qx = camera[1];
		T qy = camera[2];
		T qz = camera[3];
		T q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
		qw /= q_norm;
		qx /= q_norm;
		qy /= q_norm;
		qz /= q_norm;

		T R11 = T(1.0) - T(2)*qy*qy - T(2)*qz*qz; 		T R12 = T(2)*qx*qy - T(2)*qz*qw;			T R13 = T(2)*qx*qz + T(2)*qy*qw;
		T R21 = T(2)*qx*qy + T(2)*qz*qw;				T R22 = T(1.0) - T(2)*qx*qx - T(2)*qz*qz;	T R23 = T(2)*qz*qy - T(2)*qx*qw;
		T R31 = T(2)*qx*qz - T(2)*qy*qw;				T R32 = T(2)*qy*qz + T(2)*qx*qw;			T R33 = T(1.0) - T(2)*qx*qx - T(2)*qy*qy;


		// Set translation
		T C1 = camera[4];
		T C2 = camera[5];
		T C3 = camera[6];

		T X1 = point[0];
		T X2 = point[1];
		T X3 = point[2];



		//cout << observed_x << " " << observed_x << endl;
		//cout << X1 << " " << X2 << " " << X3 << endl;

		//cout << R11 << " " << R12 << " " << R13 << endl;
		//cout << R21 << " " << R22 << " " << R23 << endl;
		//cout << R31 << " " << R32 << " " << R33 << endl;

		// Building projection 
		T RX1 = R11*X1 + R12*X2 + R13*X3;
		T RX2 = R21*X1 + R22*X2 + R23*X3;
		T RX3 = R31*X1 + R32*X2 + R33*X3;

		T KRX1 = K11*RX1 + K12*RX2 + K13*RX3;
		T KRX2 = K21*RX1 + K22*RX2 + K23*RX3;
		T KRX3 = K31*RX1 + K32*RX2 + K33*RX3;

		T RC1 = R11*C1 + R12*C2 + R13*C3;
		T RC2 = R21*C1 + R22*C2 + R23*C3;
		T RC3 = R31*C1 + R32*C2 + R33*C3;

		T KRC1 = K11*RC1 + K12*RC2 + K13*RC3;
		T KRC2 = K21*RC1 + K22*RC2 + K23*RC3;
		T KRC3 = K31*RC1 + K32*RC2 + K33*RC3;

		T proj1 = KRX1 - KRC1;
		T proj2 = KRX2 - KRC2;
		T proj3 = KRX3 - KRC3;

		T u = proj1 / proj3;
		T v = proj2 / proj3;

		// The error is the difference between the predicted and observed position.
		residuals[0] = u - T(observed_x);
		residuals[1] = v - T(observed_y);

		//printf("%lf %lf\n", u_d, v_d);
		//printf("%lf %lf\n", observed_x, observed_y);

		return true;
	}

	// Factory to hide the construction of the CostFunction object from
	// the client code.
	static ceres::CostFunction* Create(const double observed_x,
		const double observed_y,
		const double K11,
		const double K22,
		const double K13,
		const double K23) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError4, 2, 7, 3>(
			new SnavelyReprojectionError4(observed_x, observed_y, K11, K22, K13, K23)));
	}

	double observed_x;
	double observed_y;
	double K11_;
	double K22_;
	double K13_;
	double K23_;
};




struct TriangulationRefinement {
  TriangulationRefinement(double u, double v, 
						  double p11, double p12, double p13, double p14,
						  double p21, double p22, double p23, double p24,
						  double p31, double p32, double p33, double p34)
						  : u_(u), v_(v), 
							p11_(p11), p12_(p12), p13_(p13), p14_(p14),
							p21_(p21), p22_(p22), p23_(p23), p24_(p24),
							p31_(p31), p32_(p32), p33_(p33), p34_(p34){}

  template <typename T>
  bool operator()(const T* const X, T* residuals) const {
	
	T p11 = T(p11_);	T p12 = T(p12_);	T p13 = T(p13_);	T p14 = T(p14_);
	T p21 = T(p21_);	T p22 = T(p22_);	T p23 = T(p23_);	T p24 = T(p24_);
	T p31 = T(p31_);	T p32 = T(p32_);	T p33 = T(p33_);	T p34 = T(p34_);

	T X1 = X[0];
	T X2 = X[1];
	T X3 = X[2];

	// Building projection 
	T proj1 = p11*X1+p12*X2+p13*X3+p14;
	T proj2 = p21*X1+p22*X2+p23*X3+p24;
	T proj3 = p31*X1+p32*X2+p33*X3+p34;
	
	T u = proj1/proj3;
	T v = proj2/proj3;

    // The error is the difference between the predicted and observed position.
    residuals[0] = u - T(u_);
    residuals[1] = v - T(v_);

	//cout << u_d << " " << u_ << " " << v_d << " " << v_ << endl;

    return true;
  }

 private:
  // Observations for a sample.
  const double u_;
  const double v_;

  const double p11_;	const double p12_;	const double p13_;	const double p14_;
  const double p21_;	const double p22_;	const double p23_;	const double p24_;
  const double p31_;	const double p32_;	const double p33_;	const double p34_;
};

void Triangulation_Ceres(double &x, double &y, double &z, vector<double> vu, vector<double> vv, vector<CvMat *> &vP);

void CeresSolverGoPro3(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> &cP, CvMat *X, vector<Camera> vCamera,
	double omega, double princ_x1, double princ_y1);

void CeresSolverGoPro2(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> &cP, CvMat *X, vector<Camera> vCamera,
	double omega);

void GetParameterForSBA_Distortion(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> cP, CvMat *X, CvMat *K, vector<int> visibleStructureID, BALProblem &bal_problem);
void CreateCameraMatrix1(CvMat *R, CvMat *C, CvMat *K, CvMat *P);
#endif //CERESUTILITY_H
