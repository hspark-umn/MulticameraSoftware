#include "CeresUtility.h"
#include <assert.h>
using namespace std;

void Triangulation_Ceres(double &x, double &y, double &z, vector<double> vu, vector<double> vv, vector<CvMat *> &vP)
{
	double x3d[3];
	x3d[0] = x;
	x3d[1] = y;
	x3d[2] = z;

	Problem problem;
	double err = 0;
	//cout << vP.size() << endl;
	for (int i = 0; i < vP.size(); i++) 
	{

		problem.AddResidualBlock(new AutoDiffCostFunction<TriangulationRefinement, 2, 3>(
									new TriangulationRefinement(vu[i], vv[i],
																cvGetReal2D(vP[i], 0, 0), cvGetReal2D(vP[i], 0, 1), cvGetReal2D(vP[i], 0, 2), cvGetReal2D(vP[i], 0, 3),
																cvGetReal2D(vP[i], 1, 0), cvGetReal2D(vP[i], 1, 1), cvGetReal2D(vP[i], 1, 2), cvGetReal2D(vP[i], 1, 3),
																cvGetReal2D(vP[i], 2, 0), cvGetReal2D(vP[i], 2, 1), cvGetReal2D(vP[i], 2, 2), cvGetReal2D(vP[i], 2, 3))), NULL, x3d);
	}

	//cout << err << endl;

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = false;
	//options.max_num_iterations = 200;
	//options.parameter_tolerance = 1e-14;
	//options.function_tolerance = 1e-14;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";

	x = x3d[0];
	y = x3d[1];
	z = x3d[2];

	//double err1 = 0;
	//for (int i = 0; i < vP.size(); i++) 
	//{
	//	CvMat *X3d = cvCreateMat(4,1,CV_32FC1);
	//	CvMat *x2d = cvCreateMat(3,1,CV_32FC1);
	//	cvSetReal2D(X3d, 0, 0, x);
	//	cvSetReal2D(X3d, 1, 0, y);
	//	cvSetReal2D(X3d, 2, 0, z);
	//	cvSetReal2D(X3d, 3, 0, 1);

	//	cvMatMul(vP[i], X3d, x2d);
	//	cout <<"bb" << endl;
	//	double u = cvGetReal2D(x2d, 0, 0)/cvGetReal2D(x2d, 2, 0);
	//	double v = cvGetReal2D(x2d, 1, 0)/cvGetReal2D(x2d, 2, 0);

	//	double tan_omega_half_2 = 2*tan(omega/2);
	//	double u_n = u - princ_x;
	//	double v_n = v - princ_y;

	//	double r_u = sqrt(u_n*u_n+v_n*v_n);
	//	double r_d = 1/omega*atan(r_u*tan_omega_half_2);

	//	double u_d_n = r_d/r_u * u_n;
	//	double v_d_n = r_d/r_u * v_n;

	//	double u_d = u_d_n + princ_x;
	//	double v_d = v_d_n + princ_y;

	//	err1 += sqrt((u_d-vu[i])*(u_d-vu[i])+(v_d-vv[i])*(v_d-vv[i]));

	//	cout << u_d << " " << v_d << " " << vu[i] << " " << vv[i] << "adfa" << endl;
	//	cvReleaseMat(&X3d);
	//	cvReleaseMat(&x2d);
	//}

	//cout << "Reprojection error " << err/vP.size() << " ==> " << err1/vP.size() << endl;
}

void CeresSolverUMNDome(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> &vK, vector<CvMat *> &cP, CvMat *X, vector<Camera> vCamera)
{
	PrintAlgorithm("Ceres bundle adjustment");
	vector<int> visibleStructureID;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (vFeature[iFeature].isRegistered)
		{
			visibleStructureID.push_back(vFeature[iFeature].id);
		}
	}

	BALProblem bal_problem;
	vector<int> vCameraIdx;

	GetParameterForSBA_Distortion(vFeature, vUsedFrame, cP, X, vK, visibleStructureID, vCameraIdx, bal_problem);

	//const double* observations = bal_problem.observations();
	//double tan_omega_half_2 = 2 * tan(omega / 2);
	// Create residuals for each observation in the bundle adjustment problem. The
	// parameters for cameras and points are added automatically.
	ceres::Problem problem;

	double error = 0;
	double max_error = 0;
	for (int i = 0; i < bal_problem.num_observations(); ++i)
	{
		// Each Residual block takes a point and a camera as input and outputs a 2
		// dimensional residual. Internally, the cost function stores the observed
		// image location and compares the reprojection against the observation.

		ceres::CostFunction* cost_function =
			SnavelyReprojectionError4::Create(bal_problem.observations_[2 * i + 0],
			bal_problem.observations_[2 * i + 1],
			cvGetReal2D(vK[vCameraIdx[i]], 0, 0), cvGetReal2D(vK[vCameraIdx[i]], 1, 1), cvGetReal2D(vK[vCameraIdx[i]], 0, 2), cvGetReal2D(vK[vCameraIdx[i]], 1, 2));
		problem.AddResidualBlock(cost_function,
			NULL /* squared loss */,
			bal_problem.mutable_camera_for_observation(i),
			bal_problem.mutable_point_for_observation(i));


		double measurement[2];
		measurement[0] = bal_problem.observations_[2 * i + 0];
		measurement[1] = bal_problem.observations_[2 * i + 1];
		double res[2];
		Reprojection(bal_problem.mutable_camera_for_observation(i),
			bal_problem.mutable_point_for_observation(i),
			cvGetReal2D(vK[vCameraIdx[i]], 0, 0), cvGetReal2D(vK[vCameraIdx[i]], 1, 1), cvGetReal2D(vK[vCameraIdx[i]], 0, 2), cvGetReal2D(vK[vCameraIdx[i]], 1, 2),
			measurement, res);

		error += sqrt(res[0] * res[0] + res[1] * res[1]);

		if (max_error < sqrt(res[0] * res[0] + res[1] * res[1]))
			max_error = sqrt(res[0] * res[0] + res[1] * res[1]);
	}
	error /= bal_problem.num_observations();

	// Make Ceres automatically detect the bundle structure. Note that the
	// standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
	// for standard bundle adjustment problems.
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.gradient_tolerance = 1e-8;
	options.function_tolerance = 1e-8;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = 60;
	options.num_linear_solver_threads = 60;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";


	double error_ba = 0;
	for (int i = 0; i < bal_problem.num_observations(); ++i)
	{
		double measurement[2];
		measurement[0] = bal_problem.observations_[2 * i + 0];
		measurement[1] = bal_problem.observations_[2 * i + 1];
		double res[2];
		Reprojection(bal_problem.mutable_camera_for_observation(i),
			bal_problem.mutable_point_for_observation(i),
			cvGetReal2D(vK[vCameraIdx[i]], 0, 0), cvGetReal2D(vK[vCameraIdx[i]], 1, 1), cvGetReal2D(vK[vCameraIdx[i]], 0, 2), cvGetReal2D(vK[vCameraIdx[i]], 1, 2),
			measurement, res);

		error_ba += sqrt(res[0] * res[0] + res[1] * res[1]);
	}
	error_ba /= bal_problem.num_observations();

	cout << "Reprojection error: " << error << "(" << max_error << ")" << " ==> " << error_ba << endl;

	// Retrieve data
	//for (int i = 0; i < cP.size(); i++)
	//{
	//	cvReleaseMat(&cP[i]);
	//}
	//cP.clear();
	for (int iFrame = 0; iFrame < vUsedFrame.size(); iFrame++)
	{
		CvMat *q = cvCreateMat(4, 1, CV_32FC1);
		CvMat *C = cvCreateMat(3, 1, CV_32FC1);
		cvSetReal2D(q, 0, 0, bal_problem.parameters_[7 * iFrame]);
		cvSetReal2D(q, 1, 0, bal_problem.parameters_[7 * iFrame + 1]);
		cvSetReal2D(q, 2, 0, bal_problem.parameters_[7 * iFrame + 2]);
		cvSetReal2D(q, 3, 0, bal_problem.parameters_[7 * iFrame + 3]);
		cvSetReal2D(C, 0, 0, bal_problem.parameters_[7 * iFrame + 4]);
		cvSetReal2D(C, 1, 0, bal_problem.parameters_[7 * iFrame + 5]);
		cvSetReal2D(C, 2, 0, bal_problem.parameters_[7 * iFrame + 6]);
		CvMat *R = cvCreateMat(3, 3, CV_32FC1);
		Quaternion2Rotation(q, R);
		CreateCameraMatrix1(R, C, vK[iFrame], cP[iFrame]);

		cvReleaseMat(&q);
		cvReleaseMat(&C);
		cvReleaseMat(&R);
	}

	CvMat *X_ = cvCreateMat(visibleStructureID.size(), 3, CV_32FC1);

	for (int iFeature = 0; iFeature < visibleStructureID.size(); iFeature++)
	{
		cvSetReal2D(X_, iFeature, 0, bal_problem.parameters_[7 * cP.size() + 3 * iFeature]);
		cvSetReal2D(X_, iFeature, 1, bal_problem.parameters_[7 * cP.size() + 3 * iFeature + 1]);
		cvSetReal2D(X_, iFeature, 2, bal_problem.parameters_[7 * cP.size() + 3 * iFeature + 2]);
	}
	SetIndexedMatRowwise(X, visibleStructureID, X_);
	cvReleaseMat(&X_);

	//delete[] bal_problem.point_index_;
	//   delete[] bal_problem.camera_index_;
	//   delete[] bal_problem.observations_;
	//   delete[] bal_problem.parameters_;
}


void CeresSolverGoPro3(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> &cP, CvMat *X, vector<Camera> vCamera,
	double omega, double princ_x1, double princ_y1)
{
	PrintAlgorithm("Ceres bundle adjustment");
	vector<int> visibleStructureID;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (vFeature[iFeature].isRegistered)
		{
			visibleStructureID.push_back(vFeature[iFeature].id);
		}
	}
	
	BALProblem bal_problem;
	GetParameterForSBA_Distortion(vFeature, vUsedFrame, cP, X, vCamera[0].vK[0], visibleStructureID, bal_problem);

	//const double* observations = bal_problem.observations();
	double tan_omega_half_2 = 2*tan(omega/2);
	// Create residuals for each observation in the bundle adjustment problem. The
	// parameters for cameras and points are added automatically.
	ceres::Problem problem;
	double additionaldata[8];
	additionaldata[0] = omega;
	additionaldata[1] = tan_omega_half_2;
	additionaldata[2] = princ_x1;
	additionaldata[3] = princ_y1;
	additionaldata[4] = cvGetReal2D(vCamera[0].vK[0],0,0);
	additionaldata[5] = cvGetReal2D(vCamera[0].vK[0],1,1);
	additionaldata[6] = cvGetReal2D(vCamera[0].vK[0],0,2);
	additionaldata[7] = cvGetReal2D(vCamera[0].vK[0],1,2);

	double error = 0;
	double max_error = 0;
	for (int i = 0; i < bal_problem.num_observations(); ++i) 
	{
		// Each Residual block takes a point and a camera as input and outputs a 2
		// dimensional residual. Internally, the cost function stores the observed
		// image location and compares the reprojection against the observation.

		ceres::CostFunction* cost_function =
			SnavelyReprojectionError3::Create(bal_problem.observations_[2 * i + 0],
											 bal_problem.observations_[2 * i + 1],
											 omega,
											 princ_x1, 
											 princ_y1,
											cvGetReal2D(vCamera[0].vK[0],0,0), cvGetReal2D(vCamera[0].vK[0],1,1), cvGetReal2D(vCamera[0].vK[0],0,2), cvGetReal2D(vCamera[0].vK[0],1,2));
		problem.AddResidualBlock(cost_function,
									NULL /* squared loss */,
									bal_problem.mutable_camera_for_observation(i),
									bal_problem.mutable_point_for_observation(i));


		double measurement[2];
		measurement[0] = bal_problem.observations_[2 * i + 0];
		measurement[1] = bal_problem.observations_[2 * i + 1];
		double res[2];
		Reprojection(bal_problem.mutable_camera_for_observation(i),	
					 bal_problem.mutable_point_for_observation(i), 
					 additionaldata, 
					 measurement, res);

		error += sqrt(res[0]*res[0]+res[1]*res[1]); 

		if (max_error < sqrt(res[0]*res[0]+res[1]*res[1]))
			max_error = sqrt(res[0]*res[0]+res[1]*res[1]);
	}
	error /= bal_problem.num_observations();

	// Make Ceres automatically detect the bundle structure. Note that the
	// standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
	// for standard bundle adjustment problems.
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.gradient_tolerance = 1e-8;
    options.function_tolerance = 1e-8;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = 60;
	options.num_linear_solver_threads = 60;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";


	double error_ba = 0;
	for (int i = 0; i < bal_problem.num_observations(); ++i) 
	{
		double measurement[2];
		measurement[0] = bal_problem.observations_[2 * i + 0];
		measurement[1] = bal_problem.observations_[2 * i + 1];
		double res[2];
		Reprojection(bal_problem.mutable_camera_for_observation(i),	
					 bal_problem.mutable_point_for_observation(i), 
					 additionaldata, 
					 measurement, res);

		error_ba += sqrt(res[0]*res[0]+res[1]*res[1]); 
	}
	error_ba /= bal_problem.num_observations();

	cout << "Reprojection error: " << error << "(" << max_error << ")" << " ==> " << error_ba << endl;

	// Retrieve data
	//for (int i = 0; i < cP.size(); i++)
	//{
	//	cvReleaseMat(&cP[i]);
	//}
	//cP.clear();
	for (int iFrame = 0; iFrame < vUsedFrame.size(); iFrame++)
	{
		CvMat *q = cvCreateMat(4,1,CV_32FC1);
		CvMat *C = cvCreateMat(3,1,CV_32FC1);
		cvSetReal2D(q, 0, 0, bal_problem.parameters_[7*iFrame]);
		cvSetReal2D(q, 1, 0, bal_problem.parameters_[7*iFrame+1]);
		cvSetReal2D(q, 2, 0, bal_problem.parameters_[7*iFrame+2]);
		cvSetReal2D (q, 3, 0, bal_problem.parameters_[7*iFrame+3]);
		cvSetReal2D(C, 0, 0, bal_problem.parameters_[7*iFrame+4]);
		cvSetReal2D(C, 1, 0, bal_problem.parameters_[7*iFrame+5] );
		cvSetReal2D(C, 2, 0, bal_problem.parameters_[7*iFrame+6]);
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		Quaternion2Rotation(q, R);
		CreateCameraMatrix1(R, C, vCamera[0].vK[0], cP[iFrame]);

		cvReleaseMat(&q);
		cvReleaseMat(&C);
		cvReleaseMat(&R);
	}

	CvMat *X_ = cvCreateMat(visibleStructureID.size(), 3, CV_32FC1);

	for (int iFeature = 0; iFeature < visibleStructureID.size(); iFeature++)
	{
		cvSetReal2D(X_, iFeature, 0, bal_problem.parameters_[7*cP.size()+3*iFeature]);
		cvSetReal2D(X_, iFeature, 1, bal_problem.parameters_[7*cP.size()+3*iFeature+1]);
		cvSetReal2D(X_, iFeature, 2, bal_problem.parameters_[7*cP.size()+3*iFeature+2]);
	}
	SetIndexedMatRowwise(X, visibleStructureID, X_);
	cvReleaseMat(&X_);

	//delete[] bal_problem.point_index_;
 //   delete[] bal_problem.camera_index_;
 //   delete[] bal_problem.observations_;
 //   delete[] bal_problem.parameters_;
}

void CeresSolverGoPro2(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> &cP, CvMat *X, vector<Camera> vCamera,
	double omega)
{
	PrintAlgorithm("Ceres bundle adjustment");
	vector<int> visibleStructureID;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (vFeature[iFeature].isRegistered)
		{
			visibleStructureID.push_back(vFeature[iFeature].id);
		}
	}
	
	BALProblem bal_problem;
	GetParameterForSBA_Distortion(vFeature, vUsedFrame, cP, X, vCamera[0].vK[0], visibleStructureID, bal_problem);

	//const double* observations = bal_problem.observations();
	double tan_omega_half_2 = 2*tan(omega/2);
	// Create residuals for each observation in the bundle adjustment problem. The
	// parameters for cameras and points are added automatically.
	ceres::Problem problem;
	double additionaldata[8];
	additionaldata[0] = omega;
	additionaldata[1] = tan_omega_half_2;
	additionaldata[4] = cvGetReal2D(vCamera[0].vK[0],0,0);
	additionaldata[5] = cvGetReal2D(vCamera[0].vK[0],1,1);
	additionaldata[6] = cvGetReal2D(vCamera[0].vK[0],0,2);
	additionaldata[7] = cvGetReal2D(vCamera[0].vK[0],1,2);

	double error = 0;
	double max_error = 0;
	for (int i = 0; i < bal_problem.num_observations(); ++i) 
	{
		// Each Residual block takes a point and a camera as input and outputs a 2
		// dimensional residual. Internally, the cost function stores the observed
		// image location and compares the reprojection against the observation.

		ceres::CostFunction* cost_function =
			SnavelyReprojectionError1::Create(bal_problem.observations_[2 * i + 0],
											 bal_problem.observations_[2 * i + 1],
											 omega, 
											cvGetReal2D(vCamera[0].vK[0],0,0), cvGetReal2D(vCamera[0].vK[0],1,1), cvGetReal2D(vCamera[0].vK[0],0,2), cvGetReal2D(vCamera[0].vK[0],1,2));
		problem.AddResidualBlock(cost_function,
									NULL /* squared loss */,
									bal_problem.mutable_camera_for_observation(i),
									bal_problem.mutable_point_for_observation(i));


		double measurement[2];
		measurement[0] = bal_problem.observations_[2 * i + 0];
		measurement[1] = bal_problem.observations_[2 * i + 1];
		double res[2];
		Reprojection1(bal_problem.mutable_camera_for_observation(i),	
					 bal_problem.mutable_point_for_observation(i), 
					 additionaldata, 
					 measurement, res);

		error += sqrt(res[0]*res[0]+res[1]*res[1]); 

		if (max_error < sqrt(res[0]*res[0]+res[1]*res[1]))
			max_error = sqrt(res[0]*res[0]+res[1]*res[1]);
	}
	error /= bal_problem.num_observations();

	// Make Ceres automatically detect the bundle structure. Note that the
	// standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
	// for standard bundle adjustment problems.
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = false;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";


	double error_ba = 0;
	for (int i = 0; i < bal_problem.num_observations(); ++i) 
	{
		double measurement[2];
		measurement[0] = bal_problem.observations_[2 * i + 0];
		measurement[1] = bal_problem.observations_[2 * i + 1];
		double res[2];
		Reprojection1(bal_problem.mutable_camera_for_observation(i),	
					 bal_problem.mutable_point_for_observation(i), 
					 additionaldata, 
					 measurement, res);

		error_ba += sqrt(res[0]*res[0]+res[1]*res[1]); 
	}
	error_ba /= bal_problem.num_observations();

	cout << "Reprojection error: " << error << "(" << max_error << ")" << " ==> " << error_ba << endl;

	// Retrieve data
	for (int i = 0; i < cP.size(); i++)
	{
		cvReleaseMat(&cP[i]);
	}
	cP.clear();
	for (int iFrame = 0; iFrame < vUsedFrame.size(); iFrame++)
	{
		CvMat *q = cvCreateMat(4,1,CV_32FC1);
		CvMat *C = cvCreateMat(3,1,CV_32FC1);
		cvSetReal2D(q, 0, 0, bal_problem.parameters_[7*iFrame]);
		cvSetReal2D(q, 1, 0, bal_problem.parameters_[7*iFrame+1]);
		cvSetReal2D(q, 2, 0, bal_problem.parameters_[7*iFrame+2]);
		cvSetReal2D (q, 3, 0, bal_problem.parameters_[7*iFrame+3]);
		cvSetReal2D(C, 0, 0, bal_problem.parameters_[7*iFrame+4]);
		cvSetReal2D(C, 1, 0, bal_problem.parameters_[7*iFrame+5] );
		cvSetReal2D(C, 2, 0, bal_problem.parameters_[7*iFrame+6]);
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		Quaternion2Rotation(q, R);
		CvMat *P = cvCreateMat(3,4,CV_32FC1);
		CreateCameraMatrix1(R, C, vCamera[0].vK[0], P);
		cP.push_back(P);

		cvReleaseMat(&q);
		cvReleaseMat(&C);
		cvReleaseMat(&R);
	}

	CvMat *X_ = cvCreateMat(visibleStructureID.size(), 3, CV_32FC1);

	for (int iFeature = 0; iFeature < visibleStructureID.size(); iFeature++)
	{
		cvSetReal2D(X_, iFeature, 0, bal_problem.parameters_[7*cP.size()+3*iFeature]);
		cvSetReal2D(X_, iFeature, 1, bal_problem.parameters_[7*cP.size()+3*iFeature+1]);
		cvSetReal2D(X_, iFeature, 2, bal_problem.parameters_[7*cP.size()+3*iFeature+2]);
	}
	SetIndexedMatRowwise(X, visibleStructureID, X_);
	cvReleaseMat(&X_);

	//delete[] bal_problem.point_index_;
 //   delete[] bal_problem.camera_index_;
 //   delete[] bal_problem.observations_;
 //   delete[] bal_problem.parameters_;
}


void Reprojection(double *camera, double *point, double *add, double *measurement, double *res)
{
		// Set intrinsic parameter
	double omega = add[0];
	double tan_omega_half_2 = add[1];
	double princ_x1 = add[2];
	double princ_y1 = add[3];
	double K11 = add[4];
	double K22 = add[5];
	double K13 = add[6];
	double K23 = add[7];

	double K12 = double(0);
	double K21 = double(0);
	double K31 = double(0);
	double K32 = double(0);
	double K33 = double(1);

	// Sedouble orientation
	double qw = camera[0];
	double qx = camera[1];
	double qy = camera[2];
	double qz = camera[3];
	double q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
	qw /= q_norm;
	qx /= q_norm;
	qy /= q_norm;
	qz /= q_norm;

	double R11 = double(1.0)-double(2)*qy*qy-double(2)*qz*qz; 		double R12 = double(2)*qx*qy-double(2)*qz*qw;				double R13 = double(2)*qx*qz+double(2)*qy*qw;
	double R21 = double(2)*qx*qy+double(2)*qz*qw;					double R22 = double(1.0)-double(2)*qx*qx-double(2)*qz*qz;	double R23 = double(2)*qz*qy-double(2)*qx*qw;
	double R31 = double(2)*qx*qz-double(2)*qy*qw;					double R32 = double(2)*qy*qz+double(2)*qx*qw;				double R33 = double(1.0)-double(2)*qx*qx-double(2)*qy*qy;


	// Sedouble translation
	double C1 = camera[4];
	double C2 = camera[5];
	double C3 = camera[6];

	double X1 = point[0];
	double X2 = point[1];
	double X3 = point[2];


	
	//coudouble << observed_x << " " << observed_x << endl;
	//coudouble << X1 << " " << X2 << " " << X3 << endl;

	//coudouble << R11 << " " << R12 << " " << R13 << endl;
	//coudouble << R21 << " " << R22 << " " << R23 << endl;
	//coudouble << R31 << " " << R32 << " " << R33 << endl;

	// Building projection 
	double RX1 = R11*X1+R12*X2+R13*X3;
	double RX2 = R21*X1+R22*X2+R23*X3;
	double RX3 = R31*X1+R32*X2+R33*X3;

	double KRX1 = K11*RX1+K12*RX2+K13*RX3;
	double KRX2 = K21*RX1+K22*RX2+K23*RX3;
	double KRX3 = K31*RX1+K32*RX2+K33*RX3;

	double RC1 = R11*C1+R12*C2+R13*C3;
	double RC2 = R21*C1+R22*C2+R23*C3;
	double RC3 = R31*C1+R32*C2+R33*C3;

	double KRC1 = K11*RC1+K12*RC2+K13*RC3;
	double KRC2 = K21*RC1+K22*RC2+K23*RC3;
	double KRC3 = K31*RC1+K32*RC2+K33*RC3;


	double proj1 = KRX1-KRC1;
	double proj2 = KRX2-KRC2;
	double proj3 = KRX3-KRC3;

	double u = proj1/proj3;
	double v = proj2/proj3;


	double u_n = u - princ_x1;
	double v_n = v - princ_y1;

	double r_u = sqrt(u_n*u_n+v_n*v_n);
	double r_d = double(1)/omega*atan(r_u*tan_omega_half_2);

	double u_d_n = r_d/r_u * u_n;
	double v_d_n = r_d/r_u * v_n;

	double u_d = u_d_n + princ_x1;
	double v_d = v_d_n + princ_y1;

    // The error is the difference between the predicted and observed position.
    res[0] = u_d - double(measurement[0]);
    res[1] = v_d - double(measurement[1]);
	//cout << u_d << " " << measurement[0] << " " << v_d << " " << measurement[1] << endl;
}


void Reprojection(double *camera, double *point, double K11, double K22, double K13, double K23, double *measurement, double *res)
{
	// Set intrinsic parameter

	double K12 = double(0);
	double K21 = double(0);
	double K31 = double(0);
	double K32 = double(0);
	double K33 = double(1);

	// Sedouble orientation
	double qw = camera[0];
	double qx = camera[1];
	double qy = camera[2];
	double qz = camera[3];
	double q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
	qw /= q_norm;
	qx /= q_norm;
	qy /= q_norm;
	qz /= q_norm;

	double R11 = double(1.0) - double(2)*qy*qy - double(2)*qz*qz; 		double R12 = double(2)*qx*qy - double(2)*qz*qw;				double R13 = double(2)*qx*qz + double(2)*qy*qw;
	double R21 = double(2)*qx*qy + double(2)*qz*qw;					double R22 = double(1.0) - double(2)*qx*qx - double(2)*qz*qz;	double R23 = double(2)*qz*qy - double(2)*qx*qw;
	double R31 = double(2)*qx*qz - double(2)*qy*qw;					double R32 = double(2)*qy*qz + double(2)*qx*qw;				double R33 = double(1.0) - double(2)*qx*qx - double(2)*qy*qy;


	// Sedouble translation
	double C1 = camera[4];
	double C2 = camera[5];
	double C3 = camera[6];

	double X1 = point[0];
	double X2 = point[1];
	double X3 = point[2];



	//coudouble << observed_x << " " << observed_x << endl;
	//coudouble << X1 << " " << X2 << " " << X3 << endl;

	//coudouble << R11 << " " << R12 << " " << R13 << endl;
	//coudouble << R21 << " " << R22 << " " << R23 << endl;
	//coudouble << R31 << " " << R32 << " " << R33 << endl;

	// Building projection 
	double RX1 = R11*X1 + R12*X2 + R13*X3;
	double RX2 = R21*X1 + R22*X2 + R23*X3;
	double RX3 = R31*X1 + R32*X2 + R33*X3;

	double KRX1 = K11*RX1 + K12*RX2 + K13*RX3;
	double KRX2 = K21*RX1 + K22*RX2 + K23*RX3;
	double KRX3 = K31*RX1 + K32*RX2 + K33*RX3;

	double RC1 = R11*C1 + R12*C2 + R13*C3;
	double RC2 = R21*C1 + R22*C2 + R23*C3;
	double RC3 = R31*C1 + R32*C2 + R33*C3;

	double KRC1 = K11*RC1 + K12*RC2 + K13*RC3;
	double KRC2 = K21*RC1 + K22*RC2 + K23*RC3;
	double KRC3 = K31*RC1 + K32*RC2 + K33*RC3;


	double proj1 = KRX1 - KRC1;
	double proj2 = KRX2 - KRC2;
	double proj3 = KRX3 - KRC3;

	double u = proj1 / proj3;
	double v = proj2 / proj3;

	// The error is the difference between the predicted and observed position.
	res[0] = u - double(measurement[0]);
	res[1] = v - double(measurement[1]);
	//cout << u_d << " " << measurement[0] << " " << v_d << " " << measurement[1] << endl;
}

void Reprojection1(double *camera, double *point, double *add, double *measurement, double *res)
{
		// Set intrinsic parameter
	double omega = add[0];
	double tan_omega_half_2 = add[1];
	double K11 = add[4];
	double K22 = add[5];
	double K13 = add[6];
	double K23 = add[7];

	double K12 = double(0);
	double K21 = double(0);
	double K31 = double(0);
	double K32 = double(0);
	double K33 = double(1);

	// Sedouble orientation
	double qw = camera[0];
	double qx = camera[1];
	double qy = camera[2];
	double qz = camera[3];
	double q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
	qw /= q_norm;
	qx /= q_norm;
	qy /= q_norm;
	qz /= q_norm;

	double R11 = double(1.0)-double(2)*qy*qy-double(2)*qz*qz; 		double R12 = double(2)*qx*qy-double(2)*qz*qw;				double R13 = double(2)*qx*qz+double(2)*qy*qw;
	double R21 = double(2)*qx*qy+double(2)*qz*qw;					double R22 = double(1.0)-double(2)*qx*qx-double(2)*qz*qz;	double R23 = double(2)*qz*qy-double(2)*qx*qw;
	double R31 = double(2)*qx*qz-double(2)*qy*qw;					double R32 = double(2)*qy*qz+double(2)*qx*qw;				double R33 = double(1.0)-double(2)*qx*qx-double(2)*qy*qy;


	// Sedouble translation
	double C1 = camera[4];
	double C2 = camera[5];
	double C3 = camera[6];

	double X1 = point[0];
	double X2 = point[1];
	double X3 = point[2];


	
	//coudouble << observed_x << " " << observed_x << endl;
	//coudouble << X1 << " " << X2 << " " << X3 << endl;

	//coudouble << R11 << " " << R12 << " " << R13 << endl;
	//coudouble << R21 << " " << R22 << " " << R23 << endl;
	//coudouble << R31 << " " << R32 << " " << R33 << endl;

	// Building projection 
	double RX1 = R11*X1+R12*X2+R13*X3;
	double RX2 = R21*X1+R22*X2+R23*X3;
	double RX3 = R31*X1+R32*X2+R33*X3;

	double KRX1 = K11*RX1+K12*RX2+K13*RX3;
	double KRX2 = K21*RX1+K22*RX2+K23*RX3;
	double KRX3 = K31*RX1+K32*RX2+K33*RX3;

	double RC1 = R11*C1+R12*C2+R13*C3;
	double RC2 = R21*C1+R22*C2+R23*C3;
	double RC3 = R31*C1+R32*C2+R33*C3;

	double KRC1 = K11*RC1+K12*RC2+K13*RC3;
	double KRC2 = K21*RC1+K22*RC2+K23*RC3;
	double KRC3 = K31*RC1+K32*RC2+K33*RC3;


	double proj1 = KRX1-KRC1;
	double proj2 = KRX2-KRC2;
	double proj3 = KRX3-KRC3;

	double u = proj1/proj3;
	double v = proj2/proj3;


	double u_n = (u - K13)/K11;
	double v_n = (v - K23)/K22;

	double r_u = sqrt(u_n*u_n+v_n*v_n);
	double r_d = double(1)/omega*atan(r_u*tan_omega_half_2);

	double u_d_n = r_d/r_u * u_n;
	double v_d_n = r_d/r_u * v_n;

	double u_d = u_d_n*K11 + K13;
	double v_d = v_d_n*K22 + K23;

    // The error is the difference between the predicted and observed position.
    res[0] = u_d - double(measurement[0]);
    res[1] = v_d - double(measurement[1]);
	//cout << u_d << " " << measurement[0] << " " << v_d << " " << measurement[1] << endl;
}

void CreateCameraMatrix1(CvMat *R, CvMat *C, CvMat *K, CvMat *P)
{
	cvSetIdentity(P);
	ScalarMul(C, -1, C);
	SetSubMat(P, 0,3, C);
	cvMatMul(R, P, P);
	cvMatMul(K, P, P);
}

void GetParameterForSBA_Distortion(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> cP, CvMat *X, CvMat *K, vector<int> visibleStructureID, BALProblem &bal_problem)
{
	bal_problem.num_cameras_ = vUsedFrame.size();
	bal_problem.num_points_ = visibleStructureID.size();

	int nObservations = 0;
	for (int cFeature = 0; cFeature < visibleStructureID.size(); cFeature++)
	{
		int iFeature = visibleStructureID[cFeature];
		if (!vFeature[iFeature].isRegistered)
			continue;
		for (int iVisibleFrame = 0; iVisibleFrame < vUsedFrame.size(); iVisibleFrame++)
		{
			vector<int>::iterator it = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(),vUsedFrame[iVisibleFrame]);

			if (it != vFeature[iFeature].vFrame.end())
			{
				nObservations++;
			}
		}
	}

	bal_problem.num_observations_ = nObservations;
	bal_problem.point_index_ = new int[nObservations];
    bal_problem.camera_index_ = new int[nObservations];
    bal_problem.observations_ = new double[2 * nObservations];

    bal_problem.num_parameters_ = 7 * bal_problem.num_cameras_ + 3 * bal_problem.num_points_;
    bal_problem.parameters_ = new double[bal_problem.num_parameters_];

	int observation_idx = 0;
	for (int cFeature = 0; cFeature < visibleStructureID.size(); cFeature++)
	{
		int iFeature = visibleStructureID[cFeature];
		if (!vFeature[iFeature].isRegistered)
			continue;
		for (int iVisibleFrame = 0; iVisibleFrame < vUsedFrame.size(); iVisibleFrame++)
		{
			vector<int>::iterator it = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(),vUsedFrame[iVisibleFrame]);

			if (it != vFeature[iFeature].vFrame.end())
			{
				int idx = int(it-vFeature[iFeature].vFrame.begin());
				bal_problem.camera_index_[observation_idx] = iVisibleFrame;
				bal_problem.point_index_[observation_idx] = cFeature;
				bal_problem.observations_[2*observation_idx+0] = vFeature[iFeature].vx_dis[idx];
				bal_problem.observations_[2*observation_idx+1] = vFeature[iFeature].vy_dis[idx];
				observation_idx++;
			}
		}
	}

	int nFrames = cP.size(); 
	int nFeatures = X->rows;
	for (int iFrame = 0; iFrame < vUsedFrame.size(); iFrame++)
	{
		CvMat *q = cvCreateMat(4,1,CV_32FC1);
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *invK = cvCreateMat(3,3,CV_32FC1);
		CvMat *invR = cvCreateMat(3,3,CV_32FC1);
		cvInvert(K, invK);
		GetSubMatColwise(cP[iFrame], 0, 2, R);
		GetSubMatColwise(cP[iFrame], 3, 3, t);
		cvMatMul(invK, R, R);
		cvInvert(R, invR);
		cvMatMul(invK, t, t);
		cvMatMul(invR, t, t);
		ScalarMul(t, -1, t);
		
		Rotation2Quaternion(R, q);
		bal_problem.parameters_[7*iFrame+0] = cvGetReal2D(q, 0, 0);
		bal_problem.parameters_[7*iFrame+1] = cvGetReal2D(q, 1, 0);
		bal_problem.parameters_[7*iFrame+2] = cvGetReal2D(q, 2, 0);
		bal_problem.parameters_[7*iFrame+3] = cvGetReal2D(q, 3, 0);
		bal_problem.parameters_[7*iFrame+4] = cvGetReal2D(t, 0, 0);
		bal_problem.parameters_[7*iFrame+5] = cvGetReal2D(t, 1, 0);
		bal_problem.parameters_[7*iFrame+6] = cvGetReal2D(t, 2, 0);

		cvReleaseMat(&R);
		cvReleaseMat(&t);
		cvReleaseMat(&q);
		cvReleaseMat(&invK);
		cvReleaseMat(&invR);
	}
	for (int cFeature = 0; cFeature < visibleStructureID.size(); cFeature++)
	{
		int iFeature = visibleStructureID[cFeature];
		bal_problem.parameters_[7*vUsedFrame.size()+0+3*cFeature] = cvGetReal2D(X, iFeature, 0);
		bal_problem.parameters_[7*vUsedFrame.size()+1+3*cFeature] = cvGetReal2D(X, iFeature, 1);
		bal_problem.parameters_[7*vUsedFrame.size()+2+3*cFeature] = cvGetReal2D(X, iFeature, 2);
	}
}


void GetParameterForSBA_Distortion(vector<Feature> &vFeature, vector<int> vUsedFrame, vector<CvMat *> cP, CvMat *X, vector<CvMat *> vK, vector<int> visibleStructureID, vector<int> &vCameraIdx, BALProblem &bal_problem)
{
	bal_problem.num_cameras_ = vUsedFrame.size();
	bal_problem.num_points_ = visibleStructureID.size();

	int nObservations = 0;
	for (int cFeature = 0; cFeature < visibleStructureID.size(); cFeature++)
	{
		int iFeature = visibleStructureID[cFeature];
		if (!vFeature[iFeature].isRegistered)
			continue;
		for (int iVisibleFrame = 0; iVisibleFrame < vUsedFrame.size(); iVisibleFrame++)
		{
			vector<int>::iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iVisibleFrame]);

			if (it != vFeature[iFeature].vFrame.end())
			{
				nObservations++;
			}
		}
	}

	bal_problem.num_observations_ = nObservations;
	bal_problem.point_index_ = new int[nObservations];
	bal_problem.camera_index_ = new int[nObservations];
	bal_problem.observations_ = new double[2 * nObservations];

	bal_problem.num_parameters_ = 7 * bal_problem.num_cameras_ + 3 * bal_problem.num_points_;
	bal_problem.parameters_ = new double[bal_problem.num_parameters_];

	int observation_idx = 0;
	for (int cFeature = 0; cFeature < visibleStructureID.size(); cFeature++)
	{
		int iFeature = visibleStructureID[cFeature];
		if (!vFeature[iFeature].isRegistered)
			continue;
		for (int iVisibleFrame = 0; iVisibleFrame < vUsedFrame.size(); iVisibleFrame++)
		{
			vector<int>::iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iVisibleFrame]);

			if (it != vFeature[iFeature].vFrame.end())
			{
				int idx = int(it - vFeature[iFeature].vFrame.begin());
				bal_problem.camera_index_[observation_idx] = iVisibleFrame;
				bal_problem.point_index_[observation_idx] = cFeature;
				bal_problem.observations_[2 * observation_idx + 0] = vFeature[iFeature].vx_dis[idx];
				bal_problem.observations_[2 * observation_idx + 1] = vFeature[iFeature].vy_dis[idx];
				vCameraIdx.push_back(iVisibleFrame);
				observation_idx++;
			}
		}
	}

	int nFrames = cP.size();
	int nFeatures = X->rows;
	for (int iFrame = 0; iFrame < vUsedFrame.size(); iFrame++)
	{
		CvMat *q = cvCreateMat(4, 1, CV_32FC1);
		CvMat *R = cvCreateMat(3, 3, CV_32FC1);
		CvMat *t = cvCreateMat(3, 1, CV_32FC1);
		CvMat *invK = cvCreateMat(3, 3, CV_32FC1);
		CvMat *invR = cvCreateMat(3, 3, CV_32FC1);
		cvInvert(vK[iFrame], invK);
		GetSubMatColwise(cP[iFrame], 0, 2, R);
		GetSubMatColwise(cP[iFrame], 3, 3, t);
		cvMatMul(invK, R, R);
		cvInvert(R, invR);
		cvMatMul(invK, t, t);
		cvMatMul(invR, t, t);
		ScalarMul(t, -1, t);

		Rotation2Quaternion(R, q);
		bal_problem.parameters_[7 * iFrame + 0] = cvGetReal2D(q, 0, 0);
		bal_problem.parameters_[7 * iFrame + 1] = cvGetReal2D(q, 1, 0);
		bal_problem.parameters_[7 * iFrame + 2] = cvGetReal2D(q, 2, 0);
		bal_problem.parameters_[7 * iFrame + 3] = cvGetReal2D(q, 3, 0);
		bal_problem.parameters_[7 * iFrame + 4] = cvGetReal2D(t, 0, 0);
		bal_problem.parameters_[7 * iFrame + 5] = cvGetReal2D(t, 1, 0);
		bal_problem.parameters_[7 * iFrame + 6] = cvGetReal2D(t, 2, 0);

		cvReleaseMat(&R);
		cvReleaseMat(&t);
		cvReleaseMat(&q);
		cvReleaseMat(&invK);
		cvReleaseMat(&invR);
	}
	for (int cFeature = 0; cFeature < visibleStructureID.size(); cFeature++)
	{
		int iFeature = visibleStructureID[cFeature];
		bal_problem.parameters_[7 * vUsedFrame.size() + 0 + 3 * cFeature] = cvGetReal2D(X, iFeature, 0);
		bal_problem.parameters_[7 * vUsedFrame.size() + 1 + 3 * cFeature] = cvGetReal2D(X, iFeature, 1);
		bal_problem.parameters_[7 * vUsedFrame.size() + 2 + 3 * cFeature] = cvGetReal2D(X, iFeature, 2);
	}
}