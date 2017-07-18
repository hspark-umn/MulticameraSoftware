// StaticReconstruction.cpp : Defines the entry point for the console application.
//
#include <cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <omp.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "CeresUtility.h"
#include "DataUtility.h"
#include "epnp.h"
//#include "MultiviewGeometryUtility.h"
#include "MathUtility.h"

#define DLT_RANSAC_THRESHOLD 10e-0
#define DLT_RANSAC_MAX_ITER 5e+2
#define F_ESTIMATION_THRESHOLD_INIT 1e-1
#define F_ESTIMATION_MAX_ITER_INIT 5e+3
#define F_ESTIMATION_THRESHOLD 1e+1
#define F_ESTIMATION_MAX_ITER 1e+3
#define MAX_FRAMES 2e+3
#define POINT_AT_INFINITY_ZERO 1e-2
#define PI 3.14159265

//#define FILE_PATH "D:/03 Works/Data/SocialDynamics/temp/pingpong_ref/"
#define FILE_PATH "C:/HS/ObstacleDetection/data/outdoor/outdoor_w1/Reference/"
#define FILE_PATH ""
//#define MIDDLE_FOLDER "reconstruction/"
//#define FILE_PATH ""
#define MIDDLE_FOLDER "reconstruction/"

int EPNP_ExtrinsicCameraParamEstimation(CvMat *X, CvMat *x, CvMat *K, CvMat *P);
int ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> vUsedFrame, CvMat *X_tot, vector<double> vOmega, vector<double> vprinc_x1, vector<double> vprinc_y1, vector<CvMat *> vK);
bool ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> &vUsedFrame
															, vector<int> &visibleStrucrtureID, CvMat *X_tot
															, vector<int> &visibleStrucrtureID_new, vector<vector<double> > &X_tot_new, 
															double omega, double princ_x1, double princ_y1, CvMat *K);
int ExcludePointAtInfinity_mem_fast(CvMat *X, CvMat *P1, CvMat *P2, CvMat *K1, CvMat *K2, vector<int> &featureID, vector<int> &excludedFeatureID, vector<vector<double> > &cX);
int ExcludePointBehindCamera_mem_fast(CvMat *X, CvMat *P1, CvMat *P2, vector<int> &featureID, vector<int> &excludedFeatureID, vector<vector<double> > &cX);
int LinearTriangulation_mem_fast(CvMat *x1, CvMat *P1, CvMat *x2, CvMat *P2, vector<int> &featureID, vector<vector<double> > &X, vector<int> &filteredFeatureID);
int DLT_ExtrinsicCameraParamEstimationWRansac_EPNP_mem(CvMat *X, CvMat *x, CvMat *K, CvMat *P, double ransacThreshold, int ransacMaxIter);
int VisibleIntersection23_Simple_fast(vector<Feature> &vFeature, int frame1);
int BilinearCameraPoseEstimation_OPENCV_mem_fast(vector<Feature> &vFeature, int initialFrame1, int initialFrame2, int max_nFrames, vector<Camera> vCamera, CvMat *P, CvMat *X);
void LinearTriangulation(CvMat *x1, CvMat *P1, CvMat *x2, CvMat *P2, CvMat *X);
void GetExtrinsicParameterFromE(CvMat *E, CvMat *x1, CvMat *x2, CvMat *P);
void SetCvMatFromVectors(vector<vector<double> > x, CvMat *X);
int VisibleIntersectionXOR3_mem_fast(vector<Feature> &vFeature, int frame1, int frame2, vector<vector<double> > &cx1, vector<vector<double> > &cx2, vector<int> &visibleID);
void VisibleIntersection_mem(vector<Feature> &vFeature, int frame1, int frame2, vector<vector<double> > &cx1, vector<vector<double> > &cx2, vector<int> &visibleFeatureID);
int ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> vUsedFrame, CvMat *X_tot, double omega, double princ_x1, double princ_y1, CvMat *K);
int VisibleIntersection23_mem_fast(vector<Feature> &vFeature, int frame1, CvMat *X, vector<vector<double> > &cx, vector<vector<double> > &cX, vector<int> &visibleID);

bool ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> &vUsedFrame
															, vector<int> &visibleStrucrtureID, CvMat *X_tot
															, vector<int> &visibleStrucrtureID_new, vector<vector<double> > &X_tot_new, 
															double omega, CvMat *K);
int ExcludePointHighReprojectionError_mem_fast_Distortion(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> vUsedFrame, CvMat *X_tot, double omega, CvMat *K);
bool ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> &vUsedFrame
	, vector<int> &visibleStrucrtureID, CvMat *X_tot
	, vector<int> &visibleStrucrtureID_new, vector<vector<double> > &X_tot_new);
int ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> vUsedFrame, CvMat *X_tot);

using namespace std;
int main ( int argc, char * argv[] )
{
	srand(time(NULL));
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Load data
	string path = FILE_PATH;
	string middleFolder = "reconstruction/";
	char temp[1000];
	//sprintf(temp, middleFolder.c_str(), atoi(argv[1]));
	//middleFolder = temp;

	string initialFile = middleFolder + "initial_frame.txt";
	string measurementFile = middleFolder + "stitchedmeasurement_static.txt";
	string measurementFile_save = middleFolder + "stitchedmeasurement_static_save.txt";
	string cameraFile = middleFolder + "camera.txt";
	string structureFile = middleFolder + "structure.txt";
	string structureFile3 = middleFolder + "structure3.txt";

	vector<Feature> vFeature, vUncalibratedFeature;
	vector<string> vFilename;
	vector<string> vCalibrationFilename;
	vector<int> vTimeoffset;
	vector<Camera> vCamera;
	int max_nFrames;

	//cout << argc << endl;
	//for (int i = 0; i < argc; i++)
	//{
	//	cout << argv[i] << endl;
	//}
	//return 0;

	Camera camera;
	int initial_frame1, initial_frame2, intial_nFrames;
	LoadInitialFileData(path + initialFile, initial_frame1, initial_frame2, intial_nFrames);
	//initial_frame1 = atoi(argv[2]);
	//initial_frame2 = atoi(argv[3]);
	//intial_nFrames = atoi(argv[4]);

	camera.id = 0;
	
	//initial_frame1 = 136;
	//initial_frame2 = 137;
	//intial_nFrames = 549;
	camera.nFrames = intial_nFrames;

	///////////////////////////////////////////////
	// Load calibration file

	for (int iFrame = 0; iFrame < camera.nFrames; iFrame++)
	{
		ifstream fin_cal;
		string calibfile = path + "calib/calib%07d.txt";
		char temp[1000];
		sprintf(temp, calibfile.c_str(), iFrame);
		calibfile = temp;
		fin_cal.open(calibfile.c_str(), ifstream::in);
		string dummy;
		int im_width, im_height;
		double focal_x, focal_y, princ_x, princ_y, omega;
		double princ_x1, princ_y1;
		fin_cal >> dummy >> im_width;
		fin_cal >> dummy >> im_height;
		fin_cal >> dummy >> focal_x;
		fin_cal >> dummy >> focal_y;
		fin_cal >> dummy >> princ_x;
		fin_cal >> dummy >> princ_y;

		CvMat *K_data = cvCreateMat(3, 3, CV_32FC1);
		cvSetIdentity(K_data);
		cvSetReal2D(K_data, 0, 0, focal_x);
		cvSetReal2D(K_data, 0, 2, princ_x);
		cvSetReal2D(K_data, 1, 1, focal_y);
		cvSetReal2D(K_data, 1, 2, princ_y);
		fin_cal.close();

		PrintMat(K_data);

		camera.vK.push_back(K_data);
		camera.vFrame.push_back(iFrame);
		camera.vTakenFrame.push_back(iFrame);
		camera.vTakenInstant.push_back(iFrame);
		camera.vk1.push_back(0);
		camera.vk2.push_back(0);
	}

	vCamera.push_back(camera);
	///////////////////////////////////////////////
	// Load correspondence file
	LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast(path+measurementFile, vCamera, vFeature, vUncalibratedFeature, max_nFrames);
	max_nFrames = camera.nFrames;
	vector<int> vFrameOrder;

/*	vector<vector<int> > vvFeatureIdx;
	vvFeatureIdx.resize(max_nFrames);
	for (int i = 0; i < max_nFrames; i++)
	{
		for (int i1 = 0; i1 < vFeature.size(); i1++)
		{
			for (int j = 0; j < vFeature[i1].vFrame.size(); j++)
			{
				vvFeatureIdx[vFeature[i1].vFrame[j]].push_back(i1);
			}	
		}

*/
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Initial pose estimation (bilinear estimation) 
	int frame1, frame2;
	frame1 = initial_frame1; frame2 = initial_frame2;
	vector<int> vTempFrame1 = vCamera[0].vFrame;
	while(!vTempFrame1.empty())
	{
		int randk = rand()%vTempFrame1.size();
		vFrameOrder.push_back(vTempFrame1[randk]);
		vTempFrame1.erase(vTempFrame1.begin()+randk);
	}

	CvMat *P = cvCreateMat(3, 4, CV_32FC1);
	CvMat *X = cvCreateMat(vFeature.size(), 3, CV_32FC1);
	cvSetZero(X);
	CvMat *P0 = cvCreateMat(3, 4, CV_32FC1);	cvSetIdentity(P0);
	cvMatMul(vCamera[0].vK[frame1], P0, P0);
	//vCamera[0].K = vCamera[0].vK[0];
	vector<CvMat *> cP;
	cout << "Two first frames: " << frame1 << " " << frame2 <<endl;

	int nStr = BilinearCameraPoseEstimation_OPENCV_mem_fast(vFeature, frame1, frame2, max_nFrames, vCamera, P, X);
	cout << "Number of features to do bilinear camera pose estimation: " << nStr << endl;
	cP.push_back(P0);
	cP.push_back(cvCloneMat(P));
	//return 0;

	vector<CvMat *> vK;
	vector<int> vUsedFrame;
	vUsedFrame.push_back(frame1);
	vUsedFrame.push_back(frame2);
	vK.push_back(vCamera[0].vK[frame1]);
	vK.push_back(vCamera[0].vK[frame2]);

	//SaveCameraData_KRT(path+cameraFile, cP, vCamera, vUsedFrame, max_nFrames);
	//SaveStructureData_RGB_fast(path+structureFile, X, vFeature);
	//ExcludePointHighReprojectionError_mem_fast(vFeature, cP, vUsedFrame, X);
	//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
	//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion_ObstacleDetection(vFeature, vUsedFrame, cP, X, vCamera, omega, princ_x1, princ_y1);

	//ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X, omega, princ_x1, princ_y1, vCamera[0].K);
	//ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
	

	//return 0;
	//SparseBundleAdjustment_MOTSTR_mem_fast(vFeature, vUsedFrame, cP, X, vCamera);

	cout << cP.size() << endl;
	SaveCameraData_KRT(path+cameraFile, cP, vCamera, vUsedFrame, max_nFrames);
	SaveStructureData_RGB_fast(path+structureFile, X, vFeature);

	CeresSolverUMNDome(vFeature, vUsedFrame, vK, cP, X, vCamera);


	//return 0;
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Incremental bundle adjustment
	PrintAlgorithm("Incremental Bundle Adjustment");
	int iterF = 0;
	int piCamera = -1;
	bool isFirst = true;
	bool isFailed = false;
	vector<int> failedFrame;
	while(vFrameOrder.size() != 0)
	{
		int cFrame, tempFrame = 0;
		int max_intersection = 0;
		int max_intersection_frame = -1;

		cout << "Reconstructed cameras: " << cP.size() << "/" << max_nFrames << endl;
		if (!isFailed)
		{
			for (int iFrame = 0; iFrame < failedFrame.size(); iFrame++)
			{
				vFrameOrder.push_back(failedFrame[iFrame]);
			}
		}
		//while(1)
		//{
		//	if (tempFrame == vFrameOrder.size())
		//	{ 
		//		break;
		//	}
		//	cFrame = vFrameOrder[tempFrame];
		//	tempFrame++;
		//	vector<int>::iterator it = find(vUsedFrame.begin(),vUsedFrame.end(),cFrame);
		//	if (it != vUsedFrame.end())
		//		continue;
		//	int iCam = (int) (double)cFrame/max_nFrames;

		//	int nVisible = VisibleIntersection23_Simple_fast(vFeature, cFrame);
		//	if (max_intersection < nVisible)
		//	{
		//		max_intersection = nVisible;
		//		max_intersection_frame = cFrame;
		//		//cout << "Max_intersection: " << max_intersection_frame << " " << max_intersection << endl;
		//	}	
		//	if (max_intersection > 400)
		//		break;

		//	//if ((double)max_intersection/(double) (nVisible) > 0.7)
		//	//	break;
		//}

		vector<int> vIntersection, vIntersection_frame;
		vIntersection.resize(vFrameOrder.size(),0);
		vIntersection_frame.resize(vFrameOrder.size());
		bool isOn = false;
	
		vector<Feature> vFeature_Inter;
		for (int i = 0; i < vFeature.size(); i++)
		{
			if (vFeature[i].isRegistered)
				vFeature_Inter.push_back(vFeature[i]);
		}
		cout << vFeature_Inter.size() << endl;
	
		#pragma omp parallel for
		for (int itemp = 0; itemp < vFrameOrder.size(); itemp++)
		{
			vIntersection[itemp] = 0;
			vIntersection_frame[itemp] = -1;
			if (isOn)
				continue;
			int cFrame1 = vFrameOrder[itemp];
			vector<int>::iterator it = find(vUsedFrame.begin(),vUsedFrame.end(),cFrame1);
			if (it != vUsedFrame.end())
				continue;
			
			int iCam = (int) (double)cFrame/max_nFrames;

			int nVisible = VisibleIntersection23_Simple_fast(vFeature_Inter, cFrame1);
			vIntersection[itemp] = nVisible;
			vIntersection_frame[itemp] = cFrame1;
			if (nVisible > 400)
				isOn = true;
		}

		for (int itemp = 0; itemp < vIntersection.size(); itemp++)
		{
			if (vIntersection[itemp]>max_intersection)
			{
				max_intersection = vIntersection[itemp];
				max_intersection_frame = vIntersection_frame[itemp];
			}
			if (max_intersection > 400)
				break;
		}
		cFrame = max_intersection_frame;
		iterF++;
		piCamera = (int) (double)cFrame/max_nFrames;

		if ((vFrameOrder.size() > MAX_FRAMES) || (max_intersection_frame == -1) || (max_intersection <= 40))
		{
			cout << "Final Reconstructed cameras: ";
			for (int i = 0; i < vUsedFrame.size(); i++)
				cout << vUsedFrame[i] << " ";
			cout << endl;
			break;
		}

		cout << endl <<  "-------------" << endl;
		vector<int>::const_iterator it = find(vCamera[(int)((double)cFrame/max_nFrames)].vTakenFrame.begin(), vCamera[(int)((double)cFrame/max_nFrames)].vTakenFrame.end(), cFrame%max_nFrames);
		if (it == vCamera[(int)((double)cFrame/max_nFrames)].vTakenFrame.end())
		{
			return 0;
		}
		int iTakenFrame = (int) (it - vCamera[(int)((double)cFrame/max_nFrames)].vTakenFrame.begin());
		cout << "Processing " << vCamera[(int)((double)cFrame/max_nFrames)].id << " camera " << vCamera[(int)((double)cFrame/max_nFrames)].vTakenFrame[iTakenFrame] << " ("<< cFrame << ") " << "th frame ..." << endl;
		//CvMat cx, cX, P1;
		
		vector<int> visibleID;
		vector<int> tempFrameOrder;
		for (int iFrameOrder = 0; iFrameOrder < vFrameOrder.size(); iFrameOrder++)
		{
			if (vFrameOrder[iFrameOrder] != cFrame)
				tempFrameOrder.push_back(vFrameOrder[iFrameOrder]);
		}
		vFrameOrder = tempFrameOrder;

		vector<vector<double> > cx_vec, cX_vec;
		if (!VisibleIntersection23_mem_fast(vFeature, cFrame, X, cx_vec, cX_vec, visibleID))
		{
			vFrameOrder.push_back(cFrame);
			cout << "Not enough 2D-3D correpondences" << endl;
			continue;
		}
		cout << "The number of 2D-3D correspondences: " << visibleID.size() << endl;
		CvMat *cx = cvCreateMat(cx_vec.size(), cx_vec[0].size(), CV_32FC1);
		CvMat *cX = cvCreateMat(cX_vec.size(), cX_vec[0].size(), CV_32FC1);
		SetCvMatFromVectors(cx_vec, cx);
		SetCvMatFromVectors(cX_vec, cX);
		cx_vec.clear();
		cX_vec.clear();
		CvMat *P1 = cvCreateMat(3,4,CV_32FC1);
		if (!DLT_ExtrinsicCameraParamEstimationWRansac_EPNP_mem(cX, cx, vCamera[(int)((double)cFrame/max_nFrames)].vK[iTakenFrame], P1, DLT_RANSAC_THRESHOLD, DLT_RANSAC_MAX_ITER))
		{
			//vFrameOrder.push_back(cFrame);
			cout << "Not enough inliers for ePnP" << endl;
			isFailed = true;
			failedFrame.push_back(piCamera);
			cvReleaseMat(&cx);
			cvReleaseMat(&cX);
			cvReleaseMat(&P1);
			continue;
		}
		else
		{
			isFailed = false;
			failedFrame.clear();
		}

		cvReleaseMat(&cx);
		cvReleaseMat(&cX);
		vUsedFrame.push_back(cFrame);
		vK.push_back(vCamera[0].vK[cFrame]);
		cP.push_back(P1);
		//PrintMat(P1);

		//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X, omega, princ_x1, princ_y1, vCamera[0].K);
		//cout << "Number of cameras: " << cP.size() << endl;
		//cout << "Number of structure: " << nStructure << endl;
		//SparseBundleAdjustment_MOTSTR_mem_fast(vFeature, vUsedFrame, cP, X, vCamera);
		//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
		//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
		//CeresSolverGoPro2(vFeature, vUsedFrame, cP, X, vCamera,	omega);
		//CeresSolverGoPro3(vFeature, vUsedFrame, cP, X, vCamera,	omega, princ_x1, princ_y1);

		//SaveCameraData_KRT(path+cameraFile, cP, vCamera, vUsedFrame, max_nFrames);
		//SaveStructureData_RGB_fast(path+structureFile, X, vFeature);
		// Add 3D points
		vector<CvMat *> v_newX3;
		vector<vector<int> > v_vIdx;
		v_newX3.resize(vUsedFrame.size()-1);
		v_vIdx.resize(vUsedFrame.size()-1);

		vector<Feature> vFeature_temp;
		for (int i = 0; i < vFeature.size(); i++)
		{
			if (vFeature[i].isRegistered)
				continue;
			vector<int>::iterator it = find(vFeature[i].vFrame.begin(), vFeature[i].vFrame.end(), cFrame);
			if (it == vFeature[i].vFrame.end())
				continue;
			int idx = (int) (it - vFeature[i].vFrame.begin());
			vFeature_temp.push_back(vFeature[i]);
		}

		#pragma omp parallel for
		for (int iUsedFrame = 0; iUsedFrame < vUsedFrame.size()-1; iUsedFrame++)
		{
			// Find 2D-2D correspondences which are not registered yet	
			vector<vector<double> > cx1_vec, cx2_vec;
			vector<int> visibleID1;
			if (!VisibleIntersectionXOR3_mem_fast(vFeature_temp, cFrame, vUsedFrame[iUsedFrame], cx1_vec, cx2_vec, visibleID1))
			{
				//cout << "1 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
				continue;
			}

			vector<vector<double> > ex1_vec, ex2_vec;
			vector<int> eVisibleID;
			ex1_vec = cx1_vec;
			ex2_vec = cx2_vec;
			eVisibleID = visibleID1;

			//CvMat *cx1 = cvCreateMat(cx1_vec.size(), cx1_vec[0].size(), CV_32FC1);
			//CvMat *cx2 = cvCreateMat(cx2_vec.size(), cx2_vec[0].size(), CV_32FC1);

			//SetCvMatFromVectors(cx1_vec, cx1);
			//SetCvMatFromVectors(cx2_vec, cx2);
			//cx1_vec.clear();
			//cx2_vec.clear();

			//// Exclude 2D points which do not satisfy the epipolar constraint 
			//if (!ExcludeOutliers_mem_fast(cx1, P1, cx2, cP[iUsedFrame], vCamera[(int)((double)cFrame/max_nFrames)].vK[iTakenFrame], F_ESTIMATION_THRESHOLD, visibleID, ex1_vec, ex2_vec, eVisibleID))
			//{
			//	//cout << "2 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
			//	cvReleaseMat(&cx1);
			//	cvReleaseMat(&cx2);
			//	continue;
			//}
			//cvReleaseMat(&cx1);
			//cvReleaseMat(&cx2);

			CvMat *ex1 = cvCreateMat(ex1_vec.size(), ex1_vec[0].size(), CV_32FC1);
			CvMat *ex2 = cvCreateMat(ex2_vec.size(), ex2_vec[0].size(), CV_32FC1);

			SetCvMatFromVectors(ex1_vec, ex1);
			SetCvMatFromVectors(ex2_vec, ex2);
			ex1_vec.clear();
			ex2_vec.clear();

			// Linear triangulation
			vector<int> filteredFeatureIDforTriangulation;
			vector<vector<double> > newX_vec;
			if (!LinearTriangulation_mem_fast(ex1, P1, ex2, cP[iUsedFrame], eVisibleID, newX_vec, filteredFeatureIDforTriangulation))
			{
				//cout << "3 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
				cvReleaseMat(&ex1);
				cvReleaseMat(&ex2);
				continue;
			}

			/*for (int i = 0; i < newX_vec.size(); i++)
			{
				vector<int>::const_iterator it = find(eVisibleID.begin(), eVisibleID.end(), filteredFeatureIDforTriangulation[i]);
				int idx = (int)(it-eVisibleID.begin());
				double u=newX_vec[i][0];
				double v=newX_vec[i][1];
				double w=newX_vec[i][2];
				vector<double> vu, vv;
				//cout << ex1->rows << " " << idx << endl;
				vu.push_back(cvGetReal2D(ex1, idx, 0));
				vu.push_back(cvGetReal2D(ex2, idx, 0));
				vv.push_back(cvGetReal2D(ex1, idx, 1));
				vv.push_back(cvGetReal2D(ex2, idx, 1));
				vector<CvMat *> vP_temp;
				vP_temp.push_back(P1);
				vP_temp.push_back(cP[iUsedFrame]);
				//cout << "Refine" << endl;
				Triangulation_Ceres(u, v, w, vu, vv, vP_temp);
				newX_vec[i][0] = u;
				newX_vec[i][1] = v;
				newX_vec[i][2] = w;
			}

*/
			eVisibleID = filteredFeatureIDforTriangulation;

			CvMat *newX = cvCreateMat(newX_vec.size(), newX_vec[0].size(), CV_32FC1);
			SetCvMatFromVectors(newX_vec, newX);
			cvReleaseMat(&ex1);
			cvReleaseMat(&ex2);
			newX_vec.clear();

			// Exclude the points behind the cameras
			vector<int> eVisibleID1;
			vector<vector<double> > newX1_vec;
			if (!ExcludePointBehindCamera_mem_fast(newX, P1, cP[iUsedFrame], eVisibleID, eVisibleID1, newX1_vec))
			{
				//cout << "4 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
				cvReleaseMat(&newX);
				continue;
			}
			cvReleaseMat(&newX);			
			CvMat *newX1 = cvCreateMat(newX1_vec.size(), newX1_vec[0].size(), CV_32FC1);
			SetCvMatFromVectors(newX1_vec, newX1);
			newX1_vec.clear();

			// Exclude points at infinity
			vector<int>::const_iterator it  = find(vCamera[(int) (double)vUsedFrame[iUsedFrame]/max_nFrames].vTakenFrame.begin(), vCamera[(int) (double)vUsedFrame[iUsedFrame]/max_nFrames].vTakenFrame.end(), vUsedFrame[iUsedFrame]%max_nFrames);
			int idx1 = (int) (it - vCamera[(int) (double)vUsedFrame[iUsedFrame]/max_nFrames].vTakenFrame.begin());
			vector<vector<double> > newX2_vec;
			vector<int> eVisibleID2;
			if (!ExcludePointAtInfinity_mem_fast(newX1, P1, cP[iUsedFrame], vCamera[(int)((double)cFrame/max_nFrames)].vK[iTakenFrame], 
				vCamera[(int) (double)vUsedFrame[iUsedFrame]/max_nFrames].vK[idx1], eVisibleID1, eVisibleID2, newX2_vec))
			{
				cvReleaseMat(&newX1);
				//cout << "5 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
				continue;
			}
			cvReleaseMat(&newX1);	
			CvMat *newX2 = cvCreateMat(newX2_vec.size(), newX2_vec[0].size(), CV_32FC1);
			SetCvMatFromVectors(newX2_vec, newX2);
			newX2_vec.clear();

			// Exclude points with high reprojection error
			vector<int> eVisibleID3;
			vector<vector<double> > newX3_vec;
			//if (!ExcludePointHighReprojectionError_AddingFrame_mem_fast(vFeature, cP, vUsedFrame, eVisibleID2, newX2, eVisibleID3, newX3_vec))
			//{
			//	cvReleaseMat(&newX2);
			//	//cout << "6 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
			//	continue;
			//}
			//if (!ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion(vFeature, cP, vUsedFrame, eVisibleID2, newX2, eVisibleID3, newX3_vec, omega, vCamera[0].K))
			//{
			//	cvReleaseMat(&newX2);
			//	//cout << "6 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
			//	continue;
			//}	

			//cout << newX2->rows << endl;

			//if (!ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion(vFeature, cP, vUsedFrame, eVisibleID2, newX2, eVisibleID3, newX3_vec, omega, vCamera[0].K))
			//if (!ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, eVisibleID2, newX2, eVisibleID3, newX3_vec, omega, princ_x1, princ_y1, vCamera[0].K))
			if (!ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, eVisibleID2, newX2, eVisibleID3, newX3_vec))
			{
				cvReleaseMat(&newX2);
				//cout << "6 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
				continue;
			}	

			//if (!ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, eVisibleID2, newX2, eVisibleID3, newX3_vec, omega, princ_x1, princ_y1, vCamera[0].K))
			//{
			//	cvReleaseMat(&newX2);
			//	//cout << "6 No intersection from " << vUsedFrame[iUsedFrame] << "th frame" << endl;
			//	continue;
			//}	
			cvReleaseMat(&newX2);
			//CvMat *newX3 = cvCreateMat(newX3_vec.size(), newX3_vec[0].size(), CV_32FC1);
			

			v_newX3[iUsedFrame] = cvCreateMat(newX3_vec.size(), newX3_vec[0].size(), CV_32FC1);
			v_vIdx[iUsedFrame]=eVisibleID3;
			SetCvMatFromVectors(newX3_vec, v_newX3[iUsedFrame]);
			newX3_vec.clear();
			//cvReleaseMat(&newX3);
		}

		for (int iUsedFrame = 0; iUsedFrame < v_vIdx.size(); iUsedFrame++)
		{
			if (v_vIdx[iUsedFrame].size() == 0)
				continue;
			cout << "Number of features added from " << vUsedFrame[iUsedFrame] << "th frame: " << v_vIdx[iUsedFrame].size() << endl;
			SetIndexedMatRowwise(X, v_vIdx[iUsedFrame], v_newX3[iUsedFrame]);

			for (int iVisibleId = 0; iVisibleId < v_vIdx[iUsedFrame].size(); iVisibleId++)
			{
				vFeature[v_vIdx[iUsedFrame][iVisibleId]].isRegistered = true;
			}
			cvReleaseMat(&v_newX3[iUsedFrame]);
		}

		//SaveCameraData_KRT(path+cameraFile, cP, vCamera, vUsedFrame, max_nFrames);
		//SaveStructureData_RGB_fast(path+structureFile, X, vFeature);
		// Pose and structure bundle adjustment
		//cout << "cP size" << cP.size() << endl;
		if ((int)cP.size() < 200)
		{
			//int nStructure = ExcludePointHighReprojectionError_mem_fast(vFeature, cP, vUsedFrame, X);
			//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
			//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
			int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X);
			//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X, omega, princ_x1, princ_y1, vCamera[0].K);
			cout << "Number of cameras: " << cP.size() << endl;
			cout << "Number of structure: " << nStructure << endl;
			//SparseBundleAdjustment_MOTSTR_mem_fast(vFeature, vUsedFrame, cP, X, vCamera);
			//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
			//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
			//CeresSolverGoPro2(vFeature, vUsedFrame, cP, X, vCamera,	omega);
			//CeresSolverGoPro3(vFeature, vUsedFrame, cP, X, vCamera,	omega, princ_x1, princ_y1);
			CeresSolverUMNDome(vFeature, vUsedFrame, vK, cP, X, vCamera);
			//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion_ObstacleDetection(vFeature, vUsedFrame, cP, X, vCamera, omega, princ_x1, princ_y1);
		}
		else if ((int)cP.size() < 400)
		{
			if ((int)cP.size() %2 == 0)
			{
				// Exclude 3D points that have high reprojection error
				//int nStructure = ExcludePointHighReprojectionError_mem_fast(vFeature, cP, vUsedFrame, X);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
				int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X, omega, princ_x1, princ_y1, vCamera[0].K);
				cout << "Number of cameras: " << cP.size() << endl;
				cout << "Number of structure: " << nStructure << endl;
				//SparseBundleAdjustment_MOTSTR_mem_fast(vFeature, vUsedFrame, cP, X, vCamera);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
				//CeresSolverGoPro2(vFeature, vUsedFrame, cP, X, vCamera,	omega);
				CeresSolverUMNDome(vFeature, vUsedFrame, vK, cP, X, vCamera);
				//CeresSolverGoPro3(vFeature, vUsedFrame, cP, X, vCamera,	omega, princ_x1, princ_y1);;
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion_ObstacleDetection(vFeature, vUsedFrame, cP, X, vCamera, omega, princ_x1, princ_y1);
			}
		}
		else if ((int) cP.size() < 800)
		{
			if ((int)cP.size() % 10 == 0)
			{
				// Exclude 3D points that have high reprojection error
				//int nStructure = ExcludePointHighReprojectionError_mem_fast(vFeature, cP, vUsedFrame, X);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
				int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X, omega, princ_x1, princ_y1, vCamera[0].K);
				cout << "Number of structure: " << nStructure << endl;
				//SparseBundleAdjustment_MOTSTR_mem_fast(vFeature, vUsedFrame, cP, X, vCamera);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
				//CeresSolverGoPro2(vFeature, vUsedFrame, cP, X, vCamera,	omega);
				CeresSolverUMNDome(vFeature, vUsedFrame, vK, cP, X, vCamera);
				//CeresSolverGoPro3(vFeature, vUsedFrame, cP, X, vCamera,	omega, princ_x1, princ_y1);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion_ObstacleDetection(vFeature, vUsedFrame, cP, X, vCamera, omega, princ_x1, princ_y1);
			}
		}
		else if ((int) cP.size() < 1600)
		{
			if ((int)cP.size() % 20 == 0)
			{
				// Exclude 3D points that have high reprojection error
				//int nStructure = ExcludePointHighReprojectionError_mem_fast(vFeature, cP, vUsedFrame, X);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
				int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X, omega, princ_x1, princ_y1, vCamera[0].K);
				cout << "Number of structure: " << nStructure << endl;
				//SparseBundleAdjustment_MOTSTR_mem_fast(vFeature, vUsedFrame, cP, X, vCamera);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
				//CeresSolverGoPro2(vFeature, vUsedFrame, cP, X, vCamera,	omega);
				CeresSolverUMNDome(vFeature, vUsedFrame, vK, cP, X, vCamera);
				//CeresSolverGoPro3(vFeature, vUsedFrame, cP, X, vCamera,	omega, princ_x1, princ_y1);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion_ObstacleDetection(vFeature, vUsedFrame, cP, X, vCamera, omega, princ_x1, princ_y1);
			}
		}
		else 
		{
			if ((int)cP.size() % 20 == 0)
			{
				// Exclude 3D points that have high reprojection error
				//int nStructure = ExcludePointHighReprojectionError_mem_fast(vFeature, cP, vUsedFrame, X);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
				int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X);
				//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X, omega, princ_x1, princ_y1, vCamera[0].K);
				cout << "Number of structure: " << nStructure << endl;
				//SparseBundleAdjustment_MOTSTR_mem_fast(vFeature, vUsedFrame, cP, X, vCamera);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
				//CeresSolverGoPro2(vFeature, vUsedFrame, cP, X, vCamera,	omega);
				CeresSolverUMNDome(vFeature, vUsedFrame, vK, cP, X, vCamera);
				//CeresSolverGoPro3(vFeature, vUsedFrame, cP, X, vCamera,	omega, princ_x1, princ_y1);
				//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
			}
		}
		SaveCameraData_KRT(path+cameraFile, cP, vCamera, vUsedFrame, max_nFrames);
		SaveStructureData_RGB_fast(path+structureFile, X, vFeature);
		SaveStructureData_RGB_fast_3More(path+structureFile3, X, vFeature);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	// Save data
	//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);
	//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
	//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion(vFeature, cP, vUsedFrame, X, omega, vCamera[0].K);

	//int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X, omega, princ_x1, princ_y1, vCamera[0].K);
	int nStructure = ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vFeature, cP, vUsedFrame, X);
	//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion(vFeature, vUsedFrame, cP, X, vCamera, omega);
	//CeresSolverGoPro2(vFeature, vUsedFrame, cP, X, vCamera,	omega);
	CeresSolverUMNDome(vFeature, vUsedFrame, vK, cP, X, vCamera);
	//CeresSolverGoPro3(vFeature, vUsedFrame, cP, X, vCamera,	omega, princ_x1, princ_y1);
	//SparseBundleAdjustment_MOTSTR_mem_fast_Distortion_ObstacleDetection(vFeature, vUsedFrame, cP, X, vCamera, omega, princ_x1, princ_y1);

	//int nStructure = ExcludePointHighReprojectionError_mem_fast(vFeature, cP, vUsedFrame, X);
	//SparseBundleAdjustment_MOTSTR_mem_fast(vFeature, vUsedFrame, cP, X, vCamera);
	//cout << "Number of structure: " << nStructure << endl;
	SaveCameraData_KRT(path+cameraFile, cP, vCamera, vUsedFrame, max_nFrames);
	SaveStructureData_RGB_fast(path+structureFile, X, vFeature);
	SaveStructureData_RGB_fast_3More(path+structureFile3, X, vFeature);

	return 0;
}

int ExcludePointHighReprojectionError_mem_fast_Distortion(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> vUsedFrame, CvMat *X_tot, double omega, CvMat *K)
{
	CvMat *X = cvCreateMat(4, 1, CV_32FC1);
	CvMat *x = cvCreateMat(3, 1, CV_32FC1);
	int count1=0, count2=0;

	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (vFeature[iFeature].isRegistered)
		{
			count1++;
			int nProj = 0;
			for (int iP = 0; iP < cP.size(); iP++)
			{
				vector<int>:: const_iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iP]);
				if (it != vFeature[iFeature].vFrame.end())
				{
					nProj++;
				}
			}

			if (nProj == 0)
				continue;

			for (int iP = 0; iP < cP.size(); iP++)
			{
				vector<int>:: const_iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iP]);
				if (it != vFeature[iFeature].vFrame.end())
				{
					int idx = (int) (it - vFeature[iFeature].vFrame.begin());

					cvSetReal2D(X, 0, 0, cvGetReal2D(X_tot, iFeature, 0));
					cvSetReal2D(X, 1, 0, cvGetReal2D(X_tot, iFeature, 1));
					cvSetReal2D(X, 2, 0, cvGetReal2D(X_tot, iFeature, 2));

					cvSetReal2D(X, 3, 0, 1);
					cvMatMul(cP[iP], X, x);

					double u = cvGetReal2D(x, 0, 0)/cvGetReal2D(x, 2, 0);
					double v = cvGetReal2D(x, 1, 0)/cvGetReal2D(x, 2, 0);

					double tan_omega_half_2 = tan(omega/2)*2;

					double K11 = cvGetReal2D(K, 0, 0);
					double K22 = cvGetReal2D(K, 1, 1);
					double K13 = cvGetReal2D(K, 0, 2);
					double K23 = cvGetReal2D(K, 1, 2);

					double u_n = u/K11 - K13/K11;
					double v_n = v/K22 - K23/K22;

					double r_u = sqrt(u_n*u_n+v_n*v_n);
					double r_d = 1/omega*atan(r_u*tan_omega_half_2);

					double u_d_n = r_d/r_u * u_n;
					double v_d_n = r_d/r_u * v_n;

					double u1 = u_d_n*K11 + K13;
					double v1 = v_d_n*K22 + K23;

					double u0 = vFeature[iFeature].vx_dis[idx];
					double v0 = vFeature[iFeature].vy_dis[idx];


					double dist = sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1));
					//cout << dist << endl;

					if (dist > 3)
					{
						vFeature[iFeature].vFrame.erase(vFeature[iFeature].vFrame.begin()+idx);
						vFeature[iFeature].vx.erase(vFeature[iFeature].vx.begin()+idx);
						vFeature[iFeature].vy.erase(vFeature[iFeature].vy.begin()+idx);
						vFeature[iFeature].vx_dis.erase(vFeature[iFeature].vx_dis.begin()+idx);
						vFeature[iFeature].vy_dis.erase(vFeature[iFeature].vy_dis.begin()+idx);
						vFeature[iFeature].vCamera.erase(vFeature[iFeature].vCamera.begin()+idx);
						nProj--;
						if (nProj < 2)
						{
							vFeature[iFeature].isRegistered = false;
							count2++;
							break;
						}
					}

				}
			}
		}
	}
	cout << count2 << " points are deleted." << endl;
	cvReleaseMat(&X);
	cvReleaseMat(&x);

	return count1;

}


int ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> vUsedFrame, CvMat *X_tot)
{
	CvMat *X = cvCreateMat(4, 1, CV_32FC1);
	CvMat *x = cvCreateMat(3, 1, CV_32FC1);
	int count1 = 0, count2 = 0;

	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (vFeature[iFeature].isRegistered)
		{
			count1++;
			int nProj = 0;
			for (int iP = 0; iP < cP.size(); iP++)
			{
				vector<int>::const_iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iP]);
				if (it != vFeature[iFeature].vFrame.end())
				{
					nProj++;
				}
			}
			vFeature[iFeature].nProj = nProj;

			if (nProj == 0)
				continue;

			for (int iP = 0; iP < cP.size(); iP++)
			{
				vector<int>::const_iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iP]);
				if (it != vFeature[iFeature].vFrame.end())
				{
					int idx = (int)(it - vFeature[iFeature].vFrame.begin());

					cvSetReal2D(X, 0, 0, cvGetReal2D(X_tot, iFeature, 0));
					cvSetReal2D(X, 1, 0, cvGetReal2D(X_tot, iFeature, 1));
					cvSetReal2D(X, 2, 0, cvGetReal2D(X_tot, iFeature, 2));

					cvSetReal2D(X, 3, 0, 1);
					cvMatMul(cP[iP], X, x);

					double u = cvGetReal2D(x, 0, 0) / cvGetReal2D(x, 2, 0);
					double v = cvGetReal2D(x, 1, 0) / cvGetReal2D(x, 2, 0);

					//double tan_omega_half_2 = tan(omega / 2) * 2;

					//double K11 = cvGetReal2D(K, 0, 0);
					//double K22 = cvGetReal2D(K, 1, 1);
					//double K13 = cvGetReal2D(K, 0, 2);
					//double K23 = cvGetReal2D(K, 1, 2);

					//double u_n = u/K11 - K13/K11;
					//double v_n = v/K22 - K23/K22;

					//double u_n = u - princ_x1;
					//double v_n = v - princ_y1;

					//double r_u = sqrt(u_n*u_n + v_n*v_n);
					//double r_d = 1 / omega*atan(r_u*tan_omega_half_2);

					//double u_d_n = r_d / r_u * u_n;
					//double v_d_n = r_d / r_u * v_n;

					//double u1 = u_d_n + princ_x1;
					//double v1 = v_d_n + princ_y1;

					double u0 = vFeature[iFeature].vx_dis[idx];
					double v0 = vFeature[iFeature].vy_dis[idx];


					double dist = sqrt((u0 - u)*(u0 - u) + (v0 - v)*(v0 - v));

					//cout << dist << " " << dist1 << endl;


					//cout << dist << " " << u << " " << u0 << " " << v << " " << v0<< endl;

					if (dist > 5)
					{
						vFeature[iFeature].vFrame.erase(vFeature[iFeature].vFrame.begin() + idx);
						vFeature[iFeature].vx.erase(vFeature[iFeature].vx.begin() + idx);
						vFeature[iFeature].vy.erase(vFeature[iFeature].vy.begin() + idx);
						vFeature[iFeature].vx_dis.erase(vFeature[iFeature].vx_dis.begin() + idx);
						vFeature[iFeature].vy_dis.erase(vFeature[iFeature].vy_dis.begin() + idx);
						vFeature[iFeature].vCamera.erase(vFeature[iFeature].vCamera.begin() + idx);
						nProj--;
						if (nProj < 2)
						{
							vFeature[iFeature].isRegistered = false;
							count2++;
							break;
						}
					}

				}
			}

			vFeature[iFeature].nProj = nProj;
		}
	}
	cout << count2 << " points are deleted." << endl;
	cvReleaseMat(&X);
	cvReleaseMat(&x);

	return count1;

}

int ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> vUsedFrame, CvMat *X_tot, double omega, double princ_x1, double princ_y1, CvMat *K)
{
	CvMat *X = cvCreateMat(4, 1, CV_32FC1);
	CvMat *x = cvCreateMat(3, 1, CV_32FC1);
	int count1=0, count2=0;

	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (vFeature[iFeature].isRegistered)
		{
			count1++;
			int nProj = 0;
			for (int iP = 0; iP < cP.size(); iP++)
			{
				vector<int>:: const_iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iP]);
				if (it != vFeature[iFeature].vFrame.end())
				{
					nProj++;
				}
			}
			vFeature[iFeature].nProj = nProj;

			if (nProj == 0)
				continue;

			for (int iP = 0; iP < cP.size(); iP++)
			{
				vector<int>:: const_iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iP]);
				if (it != vFeature[iFeature].vFrame.end())
				{
					int idx = (int) (it - vFeature[iFeature].vFrame.begin());

					cvSetReal2D(X, 0, 0, cvGetReal2D(X_tot, iFeature, 0));
					cvSetReal2D(X, 1, 0, cvGetReal2D(X_tot, iFeature, 1));
					cvSetReal2D(X, 2, 0, cvGetReal2D(X_tot, iFeature, 2));

					cvSetReal2D(X, 3, 0, 1);
					cvMatMul(cP[iP], X, x);

					double u = cvGetReal2D(x, 0, 0)/cvGetReal2D(x, 2, 0);
					double v = cvGetReal2D(x, 1, 0)/cvGetReal2D(x, 2, 0);

					double tan_omega_half_2 = tan(omega/2)*2;

					//double K11 = cvGetReal2D(K, 0, 0);
					//double K22 = cvGetReal2D(K, 1, 1);
					//double K13 = cvGetReal2D(K, 0, 2);
					//double K23 = cvGetReal2D(K, 1, 2);

					//double u_n = u/K11 - K13/K11;
					//double v_n = v/K22 - K23/K22;

					double u_n = u - princ_x1;
					double v_n = v - princ_y1;

					double r_u = sqrt(u_n*u_n+v_n*v_n);
					double r_d = 1/omega*atan(r_u*tan_omega_half_2);

					double u_d_n = r_d/r_u * u_n;
					double v_d_n = r_d/r_u * v_n;

					double u1 = u_d_n + princ_x1;
					double v1 = v_d_n + princ_y1;

					double u0 = vFeature[iFeature].vx_dis[idx];
					double v0 = vFeature[iFeature].vy_dis[idx];


					double dist = sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1));

					//cout << dist << " " << dist1 << endl;



					//cout << dist << endl;

					if (dist > 5)
					{
						vFeature[iFeature].vFrame.erase(vFeature[iFeature].vFrame.begin()+idx);
						vFeature[iFeature].vx.erase(vFeature[iFeature].vx.begin()+idx);
						vFeature[iFeature].vy.erase(vFeature[iFeature].vy.begin()+idx);
						vFeature[iFeature].vx_dis.erase(vFeature[iFeature].vx_dis.begin()+idx);
						vFeature[iFeature].vy_dis.erase(vFeature[iFeature].vy_dis.begin()+idx);
						vFeature[iFeature].vCamera.erase(vFeature[iFeature].vCamera.begin()+idx);
						nProj--;
						if (nProj < 2)
						{
							vFeature[iFeature].isRegistered = false;
							count2++;
							break;
						}
					}

				}
			}

			vFeature[iFeature].nProj = nProj;
		}
	}
	cout << count2 << " points are deleted." << endl;
	cvReleaseMat(&X);
	cvReleaseMat(&x);

	return count1;

}
int VisibleIntersection23_mem_fast(vector<Feature> &vFeature, int frame1, CvMat *X, vector<vector<double> > &cx, vector<vector<double> > &cX, vector<int> &visibleID)
{
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (!vFeature[iFeature].isRegistered)
			continue;
		vector<double> cx_vec, cX_vec;
		vector<int>::iterator it1 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(),frame1);
		if (it1 != vFeature[iFeature].vFrame.end() )
		{
			int idx1 = int(it1-vFeature[iFeature].vFrame.begin());

			cx_vec.push_back(vFeature[iFeature].vx[idx1]);
			cx_vec.push_back(vFeature[iFeature].vy[idx1]);

			cX_vec.push_back(cvGetReal2D(X, vFeature[iFeature].id, 0));
			cX_vec.push_back(cvGetReal2D(X, vFeature[iFeature].id, 1));
			cX_vec.push_back(cvGetReal2D(X, vFeature[iFeature].id, 2));

			cx.push_back(cx_vec);
			cX.push_back(cX_vec);

			visibleID.push_back(vFeature[iFeature].id);
		}
	}

	if (cx.size() < 1)
		return 0;
	return visibleID.size();
}

int ExcludePointHighReprojectionError_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> vUsedFrame, CvMat *X_tot, vector<double> vOmega, vector<double> vprinc_x1, vector<double> vprinc_y1, vector<CvMat *> vK)
{
	CvMat *X = cvCreateMat(4, 1, CV_32FC1);
	CvMat *x = cvCreateMat(3, 1, CV_32FC1);
	int count1=0, count2=0;

	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (vFeature[iFeature].isRegistered)
		{
			count1++;
			int nProj = 0;
			for (int iP = 0; iP < cP.size(); iP++)
			{
				vector<int>:: const_iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iP]);
				if (it != vFeature[iFeature].vFrame.end())
				{
					nProj++;
				}
			}

			if (nProj == 0)
				continue;

			for (int iP = 0; iP < cP.size(); iP++)
			{
				vector<int>:: const_iterator it = find(vFeature[iFeature].vFrame.begin(), vFeature[iFeature].vFrame.end(), vUsedFrame[iP]);
				if (it != vFeature[iFeature].vFrame.end())
				{
					int idx = (int) (it - vFeature[iFeature].vFrame.begin());

					cvSetReal2D(X, 0, 0, cvGetReal2D(X_tot, iFeature, 0));
					cvSetReal2D(X, 1, 0, cvGetReal2D(X_tot, iFeature, 1));
					cvSetReal2D(X, 2, 0, cvGetReal2D(X_tot, iFeature, 2));

					cvSetReal2D(X, 3, 0, 1);
					cvMatMul(cP[iP], X, x);

					double u = cvGetReal2D(x, 0, 0)/cvGetReal2D(x, 2, 0);
					double v = cvGetReal2D(x, 1, 0)/cvGetReal2D(x, 2, 0);

					double tan_omega_half_2 = tan(vOmega[iP]/2)*2;

					//double K11 = cvGetReal2D(K, 0, 0);
					//double K22 = cvGetReal2D(K, 1, 1);
					//double K13 = cvGetReal2D(K, 0, 2);
					//double K23 = cvGetReal2D(K, 1, 2);

					//double u_n = u/K11 - K13/K11;
					//double v_n = v/K22 - K23/K22;

					double u_n = u - vprinc_x1[iP];
					double v_n = v - vprinc_y1[iP];

					double r_u = sqrt(u_n*u_n+v_n*v_n);
					double r_d = 1/vOmega[iP]*atan(r_u*tan_omega_half_2);

					double u_d_n = r_d/r_u * u_n;
					double v_d_n = r_d/r_u * v_n;

					double u1 = u_d_n + vprinc_x1[iP];
					double v1 = v_d_n + vprinc_y1[iP];

					double u0 = vFeature[iFeature].vx_dis[idx];
					double v0 = vFeature[iFeature].vy_dis[idx];


					double dist = sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1));
					//cout << dist << endl;

					if (dist > 10)
					{
						vFeature[iFeature].vFrame.erase(vFeature[iFeature].vFrame.begin()+idx);
						vFeature[iFeature].vx.erase(vFeature[iFeature].vx.begin()+idx);
						vFeature[iFeature].vy.erase(vFeature[iFeature].vy.begin()+idx);
						vFeature[iFeature].vx_dis.erase(vFeature[iFeature].vx_dis.begin()+idx);
						vFeature[iFeature].vy_dis.erase(vFeature[iFeature].vy_dis.begin()+idx);
						vFeature[iFeature].vCamera.erase(vFeature[iFeature].vCamera.begin()+idx);
						nProj--;
						if (nProj < 2)
						{
							vFeature[iFeature].isRegistered = false;
							count2++;
							break;
						}
					}

				}
			}
		}
	}
	cout << count2 << " points are deleted." << endl;
	cvReleaseMat(&X);
	cvReleaseMat(&x);

	return count1;

}

bool ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> &vUsedFrame
															, vector<int> &visibleStrucrtureID, CvMat *X_tot
															, vector<int> &visibleStrucrtureID_new, vector<vector<double> > &X_tot_new, 
															double omega, CvMat *K)
{
	visibleStrucrtureID_new.clear();
	CvMat *X = cvCreateMat(4, 1, CV_32FC1);
	CvMat *x = cvCreateMat(3, 1, CV_32FC1);
	for (int iVS = 0; iVS < visibleStrucrtureID.size(); iVS++)
	{
		bool isIn = true;
		for (int iP = 0; iP < cP.size(); iP++)
		{
			vector<int>:: const_iterator it = find(vFeature[visibleStrucrtureID[iVS]].vFrame.begin(), vFeature[visibleStrucrtureID[iVS]].vFrame.end(), vUsedFrame[iP]);
			if (it != vFeature[visibleStrucrtureID[iVS]].vFrame.end())
			{
				int idx = (int) (it - vFeature[visibleStrucrtureID[iVS]].vFrame.begin());

				cvSetReal2D(X, 0, 0, cvGetReal2D(X_tot, iVS, 0));
				cvSetReal2D(X, 1, 0, cvGetReal2D(X_tot, iVS, 1));
				cvSetReal2D(X, 2, 0, cvGetReal2D(X_tot, iVS, 2));

				cvSetReal2D(X, 3, 0, 1);
				cvMatMul(cP[iP], X, x);

				double u = cvGetReal2D(x, 0, 0)/cvGetReal2D(x, 2, 0);
				double v = cvGetReal2D(x, 1, 0)/cvGetReal2D(x, 2, 0);

				double tan_omega_half_2 = tan(omega/2)*2;

				double K11 = cvGetReal2D(K, 0, 0);
				double K22 = cvGetReal2D(K, 1, 1);
				double K13 = cvGetReal2D(K, 0, 2);
				double K23 = cvGetReal2D(K, 1, 2);

				double u_n = u/K11 - K13/K11;
				double v_n = v/K22 - K23/K22;

				double r_u = sqrt(u_n*u_n+v_n*v_n);
				double r_d = 1/omega*atan(r_u*tan_omega_half_2);

				double u_d_n = r_d/r_u * u_n;
				double v_d_n = r_d/r_u * v_n;

				double u1 = u_d_n*K11 + K13;
				double v1 = v_d_n*K22 + K23;

				double u0 = vFeature[visibleStrucrtureID[iVS]].vx_dis[idx];
				double v0 = vFeature[visibleStrucrtureID[iVS]].vy_dis[idx];
				//double u1 = cvGetReal2D(x, 0, 0)/cvGetReal2D(x, 2, 0);
				//double v1 = cvGetReal2D(x, 1, 0)/cvGetReal2D(x, 2, 0);

				//cout << sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1)) << endl;
				if (sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1)) > 3)
				{
					//cout << visibleStrucrtureID[iVS] << "th 3D point erased " << sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1)) << endl;
					isIn = false;
					break;
				}
			}
		}
		if (isIn)
		{
			visibleStrucrtureID_new.push_back(visibleStrucrtureID[iVS]);
			vector<double> X_tot_new_vec;
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 0));
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 1));
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 2));
			X_tot_new.push_back(X_tot_new_vec);
		}
	}
	cvReleaseMat(&X);
	cvReleaseMat(&x);

	if (visibleStrucrtureID_new.size() == 0)
		return false;

	return true;
}

bool ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> &vUsedFrame
															, vector<int> &visibleStrucrtureID, CvMat *X_tot
															, vector<int> &visibleStrucrtureID_new, vector<vector<double> > &X_tot_new, 
															double omega, double princ_x1, double princ_y1, CvMat *K)
{
	visibleStrucrtureID_new.clear();
	CvMat *X = cvCreateMat(4, 1, CV_32FC1);
	CvMat *x = cvCreateMat(3, 1, CV_32FC1);
	for (int iVS = 0; iVS < visibleStrucrtureID.size(); iVS++)
	{
		bool isIn = true;
		for (int iP = 0; iP < cP.size(); iP++)
		{
			vector<int>:: const_iterator it = find(vFeature[visibleStrucrtureID[iVS]].vFrame.begin(), vFeature[visibleStrucrtureID[iVS]].vFrame.end(), vUsedFrame[iP]);
			if (it != vFeature[visibleStrucrtureID[iVS]].vFrame.end())
			{
				int idx = (int) (it - vFeature[visibleStrucrtureID[iVS]].vFrame.begin());

				cvSetReal2D(X, 0, 0, cvGetReal2D(X_tot, iVS, 0));
				cvSetReal2D(X, 1, 0, cvGetReal2D(X_tot, iVS, 1));
				cvSetReal2D(X, 2, 0, cvGetReal2D(X_tot, iVS, 2));

				cvSetReal2D(X, 3, 0, 1);
				cvMatMul(cP[iP], X, x);

				double u = cvGetReal2D(x, 0, 0)/cvGetReal2D(x, 2, 0);
				double v = cvGetReal2D(x, 1, 0)/cvGetReal2D(x, 2, 0);

				double tan_omega_half_2 = tan(omega/2)*2;

				//double K11 = cvGetReal2D(K, 0, 0);
				//double K22 = cvGetReal2D(K, 1, 1);
				//double K13 = cvGetReal2D(K, 0, 2);
				//double K23 = cvGetReal2D(K, 1, 2);

				//double u_n = u/K11 - K13/K11;
				//double v_n = v/K22 - K23/K22;

				double u_n = u - princ_x1;
				double v_n = v - princ_y1;

				double r_u = sqrt(u_n*u_n+v_n*v_n);
				double r_d = 1/omega*atan(r_u*tan_omega_half_2);

				double u_d_n = r_d/r_u * u_n;
				double v_d_n = r_d/r_u * v_n;

				double u1 = u_d_n + princ_x1;
				double v1 = v_d_n + princ_y1;

				double u0 = vFeature[visibleStrucrtureID[iVS]].vx_dis[idx];
				double v0 = vFeature[visibleStrucrtureID[iVS]].vy_dis[idx];
				//double u1 = cvGetReal2D(x, 0, 0)/cvGetReal2D(x, 2, 0);
				//double v1 = cvGetReal2D(x, 1, 0)/cvGetReal2D(x, 2, 0);

				if (sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1)) > 5)
				{
					//cout << visibleStrucrtureID[iVS] << "th 3D point erased " << sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1)) << endl;
					isIn = false;
					break;
				}
			}
		}
		if (isIn)
		{
			visibleStrucrtureID_new.push_back(visibleStrucrtureID[iVS]);
			vector<double> X_tot_new_vec;
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 0));
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 1));
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 2));
			X_tot_new.push_back(X_tot_new_vec);
		}
	}
	cvReleaseMat(&X);
	cvReleaseMat(&x);

	if (visibleStrucrtureID_new.size() == 0)
		return false;

	return true;
}

bool ExcludePointHighReprojectionError_AddingFrame_mem_fast_Distortion_ObstacleDetection(vector<Feature> &vFeature, vector<CvMat *> &cP, vector<int> &vUsedFrame
	, vector<int> &visibleStrucrtureID, CvMat *X_tot
	, vector<int> &visibleStrucrtureID_new, vector<vector<double> > &X_tot_new)
{
	visibleStrucrtureID_new.clear();
	CvMat *X = cvCreateMat(4, 1, CV_32FC1);
	CvMat *x = cvCreateMat(3, 1, CV_32FC1);
	for (int iVS = 0; iVS < visibleStrucrtureID.size(); iVS++)
	{
		bool isIn = true;
		for (int iP = 0; iP < cP.size(); iP++)
		{
			vector<int>::const_iterator it = find(vFeature[visibleStrucrtureID[iVS]].vFrame.begin(), vFeature[visibleStrucrtureID[iVS]].vFrame.end(), vUsedFrame[iP]);
			if (it != vFeature[visibleStrucrtureID[iVS]].vFrame.end())
			{
				int idx = (int)(it - vFeature[visibleStrucrtureID[iVS]].vFrame.begin());

				cvSetReal2D(X, 0, 0, cvGetReal2D(X_tot, iVS, 0));
				cvSetReal2D(X, 1, 0, cvGetReal2D(X_tot, iVS, 1));
				cvSetReal2D(X, 2, 0, cvGetReal2D(X_tot, iVS, 2));

				cvSetReal2D(X, 3, 0, 1);
				cvMatMul(cP[iP], X, x);

				double u = cvGetReal2D(x, 0, 0) / cvGetReal2D(x, 2, 0);
				double v = cvGetReal2D(x, 1, 0) / cvGetReal2D(x, 2, 0);

				//double tan_omega_half_2 = tan(omega / 2) * 2;

				//double K11 = cvGetReal2D(K, 0, 0);
				//double K22 = cvGetReal2D(K, 1, 1);
				//double K13 = cvGetReal2D(K, 0, 2);
				//double K23 = cvGetReal2D(K, 1, 2);

				//double u_n = u/K11 - K13/K11;
				//double v_n = v/K22 - K23/K22;

				//double u_n = u - princ_x1;
				//double v_n = v - princ_y1;

				//double r_u = sqrt(u_n*u_n + v_n*v_n);
				//double r_d = 1 / omega*atan(r_u*tan_omega_half_2);

				//double u_d_n = r_d / r_u * u_n;
				//double v_d_n = r_d / r_u * v_n;

				//double u1 = u_d_n + princ_x1;
				//double v1 = v_d_n + princ_y1;

				double u0 = vFeature[visibleStrucrtureID[iVS]].vx_dis[idx];
				double v0 = vFeature[visibleStrucrtureID[iVS]].vy_dis[idx];
				//double u1 = cvGetReal2D(x, 0, 0)/cvGetReal2D(x, 2, 0);
				//double v1 = cvGetReal2D(x, 1, 0)/cvGetReal2D(x, 2, 0);

				if (sqrt((u0 - u)*(u0 - u) + (v0 - v)*(v0 - v)) > 5)
				{
					//cout << visibleStrucrtureID[iVS] << "th 3D point erased " << sqrt((u0-u1)*(u0-u1)+(v0-v1)*(v0-v1)) << endl;
					isIn = false;
					break;
				}
			}
		}
		if (isIn)
		{
			visibleStrucrtureID_new.push_back(visibleStrucrtureID[iVS]);
			vector<double> X_tot_new_vec;
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 0));
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 1));
			X_tot_new_vec.push_back(cvGetReal2D(X_tot, iVS, 2));
			X_tot_new.push_back(X_tot_new_vec);
		}
	}
	cvReleaseMat(&X);
	cvReleaseMat(&x);

	if (visibleStrucrtureID_new.size() == 0)
		return false;

	return true;
}

int ExcludePointAtInfinity_mem_fast(CvMat *X, CvMat *P1, CvMat *P2, CvMat *K1, CvMat *K2, vector<int> &featureID, vector<int> &excludedFeatureID, vector<vector<double> > &cX)
{
	CvMat *q1 = cvCreateMat(4,1,CV_32FC1);
	CvMat *R1 = cvCreateMat(3,3,CV_32FC1);
	CvMat *t1 = cvCreateMat(3,1,CV_32FC1);
	CvMat *invK1 = cvCreateMat(3,3,CV_32FC1);
	CvMat *invR1 = cvCreateMat(3,3,CV_32FC1);

	CvMat *q2 = cvCreateMat(4,1,CV_32FC1);
	CvMat *R2 = cvCreateMat(3,3,CV_32FC1);
	CvMat *t2 = cvCreateMat(3,1,CV_32FC1);
	CvMat *invK2 = cvCreateMat(3,3,CV_32FC1);
	CvMat *invR2 = cvCreateMat(3,3,CV_32FC1);

	GetSubMatColwise(P1, 0, 2, R1);
	GetSubMatColwise(P1, 3, 3, t1);
	cvInvert(K1, invK1);
	cvMatMul(invK1, R1, R1);
	cvInvert(R1, invR1);
	cvMatMul(invK1, t1, t1);
	cvMatMul(invR1, t1, t1);
	ScalarMul(t1, -1, t1);

	GetSubMatColwise(P2, 0, 2, R2);
	GetSubMatColwise(P2, 3, 3, t2);
	cvInvert(K2, invK2);
	cvMatMul(invK2, R2, R2);
	cvInvert(R2, invR2);
	cvMatMul(invK2, t2, t2);
	cvMatMul(invR2, t2, t2);
	ScalarMul(t2, -1, t2);

	double xC1 = cvGetReal2D(t1, 0, 0);
	double yC1 = cvGetReal2D(t1, 1, 0);
	double zC1 = cvGetReal2D(t1, 2, 0);
	double xC2 = cvGetReal2D(t2, 0, 0);
	double yC2 = cvGetReal2D(t2, 1, 0);
	double zC2 = cvGetReal2D(t2, 2, 0);

	excludedFeatureID.clear();
	vector<double> vInner;
	for (int i = 0; i < X->rows; i++)
	{
		double x3D = cvGetReal2D(X, i, 0);
		double y3D = cvGetReal2D(X, i, 1);
		double z3D = cvGetReal2D(X, i, 2);

		double v1x = x3D - xC1;		double v1y = y3D - yC1;		double v1z = z3D - zC1;
		double v2x = x3D - xC2;		double v2y = y3D - yC2;		double v2z = z3D - zC2;

		double nv1 = sqrt(v1x*v1x+v1y*v1y+v1z*v1z);
		double nv2 = sqrt(v2x*v2x+v2y*v2y+v2z*v2z);
		v1x /= nv1;		v1y /= nv1;		v1z /= nv1;
		v2x /= nv2;		v2y /= nv2;		v2z /= nv2;
		double inner = v1x*v2x+v1y*v2y+v1z*v2z;
		vInner.push_back(inner);
		if ((abs(inner) < cos(PI/180*3)) && (inner > 0))
		{			
			vector<double> cX_vec;
			cX_vec.push_back(x3D);
			cX_vec.push_back(y3D);
			cX_vec.push_back(z3D);
			cX.push_back(cX_vec);
			excludedFeatureID.push_back(featureID[i]);
		}
	}
	if (excludedFeatureID.size() == 0)
	{
		cvReleaseMat(&q1);
		cvReleaseMat(&R1);
		cvReleaseMat(&t1);
		cvReleaseMat(&invK1);
		cvReleaseMat(&invR1);

		cvReleaseMat(&q2);
		cvReleaseMat(&R2);
		cvReleaseMat(&t2);
		cvReleaseMat(&invK2);
		cvReleaseMat(&invR2);
		return 0;
	}

	cvReleaseMat(&q1);
	cvReleaseMat(&R1);
	cvReleaseMat(&t1);
	cvReleaseMat(&invK1);
	cvReleaseMat(&invR1);

	cvReleaseMat(&q2);
	cvReleaseMat(&R2);
	cvReleaseMat(&t2);
	cvReleaseMat(&invK2);
	cvReleaseMat(&invR2);
	return cX.size();
}


int ExcludePointBehindCamera_mem_fast(CvMat *X, CvMat *P1, CvMat *P2, vector<int> &featureID, vector<int> &excludedFeatureID, vector<vector<double> > &cX)
{
	//CvMat *H1 = cvCreateMat(4, 4, CV_32FC1);	CvMat *invH1 = cvCreateMat(4, 4, CV_32FC1);
	//CvMat *HX1 = cvCreateMat(X->rows, X->cols, CV_32FC1);
	//cvSetIdentity(H1);
	//SetSubMat(H1, 0, 0, P1);
	//cvInvert(H1, invH1);
	//Pxx_inhomo(H1, X, HX1);

	//CvMat *H2 = cvCreateMat(4, 4, CV_32FC1);	CvMat *invH2 = cvCreateMat(4, 4, CV_32FC1);
	//CvMat *HX2 = cvCreateMat(X->rows, X->cols, CV_32FC1);
	//cvSetIdentity(H2);
	//SetSubMat(H2, 0, 0, P2);
	//cvInvert(H2, invH2);
	//Pxx_inhomo(H2, X, HX2);

	excludedFeatureID.clear();
	for (int i = 0; i < X->rows; i++)
	{
		CvMat *X_3d = cvCreateMat(4,1,CV_32FC1);
		cvSetReal2D(X_3d, 0, 0, cvGetReal2D(X,i,0));
		cvSetReal2D(X_3d, 1, 0, cvGetReal2D(X,i,1));
		cvSetReal2D(X_3d, 2, 0, cvGetReal2D(X,i,2));
		cvSetReal2D(X_3d, 3, 0, 1);

		CvMat *x1 = cvCreateMat(3,1,CV_32FC1);
		CvMat *x2 = cvCreateMat(3,1,CV_32FC1);

		cvMatMul(P1, X_3d, x1);
		cvMatMul(P2, X_3d, x2);
		
		if ((cvGetReal2D(x1, 2, 0) > 0) && (cvGetReal2D(x2, 2, 0) > 0))
		{
			excludedFeatureID.push_back(featureID[i]);

			vector<double> cX_vec;
			cX_vec.push_back(cvGetReal2D(X, i, 0));
			cX_vec.push_back(cvGetReal2D(X, i, 1));
			cX_vec.push_back(cvGetReal2D(X, i, 2));

			cX.push_back(cX_vec);
		}

		cvReleaseMat(&x1);
		cvReleaseMat(&x2);
		cvReleaseMat(&X_3d);
	}
	if (excludedFeatureID.size() == 0)
	{
		//cvReleaseMat(&H1);
		//cvReleaseMat(&HX1);
		//cvReleaseMat(&H2);
		//cvReleaseMat(&HX2);
		//cvReleaseMat(&invH1);
		//cvReleaseMat(&invH2);
		return 0;
	}
	//cvReleaseMat(&H1);
	//cvReleaseMat(&HX1);
	//cvReleaseMat(&H2);
	//cvReleaseMat(&HX2);
	//cvReleaseMat(&invH1);
	//cvReleaseMat(&invH2);
	return cX.size();
}


int LinearTriangulation_mem_fast(CvMat *x1, CvMat *P1, CvMat *x2, CvMat *P2, vector<int> &featureID, vector<vector<double> > &X, vector<int> &filteredFeatureID)
{
	filteredFeatureID.clear();
	CvMat *A = cvCreateMat(4, 4, CV_32FC1);
	CvMat *A1 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *A2 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *A3 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *A4 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *P1_1 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *P1_2 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *P1_3 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *P2_1 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *P2_2 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *P2_3 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *temp14_1 = cvCreateMat(1, 4, CV_32FC1);
	CvMat *x = cvCreateMat(A->cols, 1, CV_32FC1);

	for (int ix = 0; ix < x1->rows; ix++)
	{
		GetSubMatRowwise(P1, 0, 0, P1_1);
		GetSubMatRowwise(P1, 1, 1, P1_2);
		GetSubMatRowwise(P1, 2, 2, P1_3);
		GetSubMatRowwise(P2, 0, 0, P2_1);
		GetSubMatRowwise(P2, 1, 1, P2_2);
		GetSubMatRowwise(P2, 2, 2, P2_3);

		ScalarMul(P1_3, cvGetReal2D(x1, ix, 0), temp14_1);
		cvSub(temp14_1, P1_1, A1);

		ScalarMul(P1_3, cvGetReal2D(x1, ix, 1), temp14_1);
		cvSub(temp14_1, P1_2, A2);

		ScalarMul(P2_3, cvGetReal2D(x2, ix, 0), temp14_1);
		cvSub(temp14_1, P2_1, A3);
		ScalarMul(P2_3, cvGetReal2D(x2, ix, 1), temp14_1);
		cvSub(temp14_1, P2_2, A4);
		SetSubMat(A, 0, 0, A1);
		SetSubMat(A, 1, 0, A2);
		SetSubMat(A, 2, 0, A3);
		SetSubMat(A, 3, 0, A4);

		LS_homogeneous(A, x);

		double v = cvGetReal2D(x, 3, 0);
		if (abs(v) < POINT_AT_INFINITY_ZERO)
		{
			continue;
		}

		vector<double> X_vec;
		X_vec.push_back(cvGetReal2D(x, 0, 0)/v);
		X_vec.push_back(cvGetReal2D(x, 1, 0)/v);
		X_vec.push_back(cvGetReal2D(x, 2, 0)/v);
		X.push_back(X_vec);
		filteredFeatureID.push_back(featureID[ix]);
	}

	cvReleaseMat(&A);
	cvReleaseMat(&A1);
	cvReleaseMat(&A2);
	cvReleaseMat(&A3);
	cvReleaseMat(&A4);
	cvReleaseMat(&P1_1);
	cvReleaseMat(&P1_2);
	cvReleaseMat(&P1_3);
	cvReleaseMat(&P2_1);
	cvReleaseMat(&P2_2);
	cvReleaseMat(&P2_3);
	cvReleaseMat(&temp14_1);
	cvReleaseMat(&x);

	if (filteredFeatureID.size() == 0)
		return 0;

	return X.size();
}

int VisibleIntersectionXOR3_mem_fast(vector<Feature> &vFeature, int frame1, int frame2, vector<vector<double> > &cx1, vector<vector<double> > &cx2, vector<int> &visibleID)
{
	visibleID.clear();
	
	/*for (int i = 0; i < vvFeatureIdx[frame1].size(); i++)
	{
		if (vFeature[vvFeatureIdx[frame1][i]].isRegistered)
                        continue;
		vector<int>::iterator it1 = find(vFeature[vvFeatureIdx[frame1][i]].vFrame.begin(),vFeature[vvFeatureIdx[frame1][i]].vFrame.end(),frame1);
                if (it1 == vFeature[vvFeatureIdx[frame1][i]].vFrame.end())
                        continue;

                vector<int>::iterator it2 = find(vFeature[vvFeatureIdx[frame1][i]].vFrame.begin(),vFeature[vvFeatureIdx[frame1][i]].vFrame.end(),frame2);
                if (it2 == vFeature[vvFeatureIdx[frame1][i]].vFrame.end())
                        continue;

                vector<double> cx1_vec, cx2_vec;
		int idx1 = int(it1-vFeature[vvFeatureIdx[frame1][i]].vFrame.begin());


                int idx2 = int(it2-vFeature[vvFeatureIdx[frame1][i]].vFrame.begin());

                cx1_vec.push_back(vFeature[vvFeatureIdx[frame1][i]].vx[idx1]);
                cx1_vec.push_back(vFeature[vvFeatureIdx[frame1][i]].vy[idx1]);

                cx2_vec.push_back(vFeature[vvFeatureIdx[frame1][i]].vx[idx2]);
                cx2_vec.push_back(vFeature[vvFeatureIdx[frame1][i]].vy[idx2]);

                cx1.push_back(cx1_vec);
                cx2.push_back(cx2_vec);
                visibleID.push_back(vFeature[vvFeatureIdx[frame1][i]].id);
	
	}
*
*/
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (vFeature[iFeature].isRegistered)
			continue;
		vector<int>::iterator it1 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(),frame1);
		if (it1 == vFeature[iFeature].vFrame.end())
			continue;
		vector<int>::iterator it2 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(),frame2);
		if (it2 == vFeature[iFeature].vFrame.end())
			continue;

		vector<double> cx1_vec, cx2_vec;

		int idx1 = int(it1-vFeature[iFeature].vFrame.begin());
		int idx2 = int(it2-vFeature[iFeature].vFrame.begin());

		cx1_vec.push_back(vFeature[iFeature].vx[idx1]);
		cx1_vec.push_back(vFeature[iFeature].vy[idx1]);

		cx2_vec.push_back(vFeature[iFeature].vx[idx2]);
		cx2_vec.push_back(vFeature[iFeature].vy[idx2]);

		cx1.push_back(cx1_vec);
		cx2.push_back(cx2_vec);
		visibleID.push_back(vFeature[iFeature].id);
	}

	if (visibleID.size() == 0)
	{
		return 0;
	}

	return cx1.size();
}

void VisibleIntersection_mem(vector<Feature> &vFeature, int frame1, int frame2, vector<vector<double> > &cx1, vector<vector<double> > &cx2, vector<int> &visibleFeatureID)
{
	vector<double> x1, y1, x2, y2;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		//if (!vFeature[iFeature].isRegistered)
		//	continue;

		vector<int>::iterator it1 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(),frame1);
		vector<int>::iterator it2 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(),frame2);

		if ((it1 != vFeature[iFeature].vFrame.end()) && (it2 != vFeature[iFeature].vFrame.end()))
		{
			int idx = int(it1-vFeature[iFeature].vFrame.begin());
			x1.push_back(vFeature[iFeature].vx[idx]);
			y1.push_back(vFeature[iFeature].vy[idx]);
			idx = int(it2-vFeature[iFeature].vFrame.begin());
			x2.push_back(vFeature[iFeature].vx[idx]);
			y2.push_back(vFeature[iFeature].vy[idx]);
			visibleFeatureID.push_back(vFeature[iFeature].id);
		}
	}
	//cout << "# intersection: " << visibleFeatureID.size() << endl;
	for (int i = 0; i < x1.size(); i++)
	{
		vector<double> x1_vec, x2_vec;
		x1_vec.push_back(x1[i]);
		x1_vec.push_back(y1[i]);

		x2_vec.push_back(x2[i]);
		x2_vec.push_back(y2[i]);

		cx1.push_back(x1_vec);
		cx2.push_back(x2_vec);
	}
}

void SetCvMatFromVectors(vector<vector<double> > x, CvMat *X)
{
	for (int i = 0; i < x.size(); i++)
	{
		for (int j = 0; j < x[i].size(); j++)
			cvSetReal2D(X, i, j, x[i][j]);
	}
}

void GetExtrinsicParameterFromE(CvMat *E, CvMat *x1, CvMat *x2, CvMat *P)
{
	CvMat *W = cvCreateMat(3, 3, CV_32FC1);
	CvMat *U = cvCreateMat(3, 3, CV_32FC1);
	CvMat *D = cvCreateMat(3, 3, CV_32FC1);
	CvMat *Vt = cvCreateMat(3, 3, CV_32FC1);
	CvMat *Wt = cvCreateMat(3, 3, CV_32FC1);
	cvSVD(E, D, U, Vt, CV_SVD_V_T);

	//cvSetReal2D(D, 1,1,cvGetReal2D(D,0,0));
	//cvMatMul(U, D, E);
	//cvMatMul(E, Vt, E);
	//cvSVD(E, D, U, Vt, CV_SVD_V_T);

	cvSetReal2D(W, 0, 0, 0);	cvSetReal2D(W, 0, 1, -1);	cvSetReal2D(W, 0, 2, 0);
	cvSetReal2D(W, 1, 0, 1);	cvSetReal2D(W, 1, 1, 0);	cvSetReal2D(W, 1, 2, 0);
	cvSetReal2D(W, 2, 0, 0);	cvSetReal2D(W, 2, 1, 0);	cvSetReal2D(W, 2, 2, 1);
	cvTranspose(W, Wt);

	CvMat *P0 = cvCreateMat(3, 4, CV_32FC1);
	cvSetIdentity(P0);

	CvMat *P1 = cvCreateMat(3, 4, CV_32FC1);
	CvMat *P2 = cvCreateMat(3, 4, CV_32FC1);
	CvMat *P3 = cvCreateMat(3, 4, CV_32FC1);
	CvMat *P4 = cvCreateMat(3, 4, CV_32FC1);

	CvMat *R1 = cvCreateMat(3, 3, CV_32FC1);
	CvMat *R2 = cvCreateMat(3, 3, CV_32FC1);
	CvMat *t1 = cvCreateMat(3, 1, CV_32FC1);
	CvMat *t2 = cvCreateMat(3, 1, CV_32FC1);
	CvMat *temp33 = cvCreateMat(3, 3, CV_32FC1);

	cvMatMul(U, W, temp33);
	cvMatMul(temp33, Vt, R1);
	cvMatMul(U, Wt, temp33);
	cvMatMul(temp33, Vt, R2);

	cvSetReal2D(t1, 0, 0, cvGetReal2D(U,0,2));
	cvSetReal2D(t1, 1, 0, cvGetReal2D(U,1,2));
	cvSetReal2D(t1, 2, 0, cvGetReal2D(U,2,2));
	ScalarMul(t1, -1, t2);

	SetSubMat(P1, 0, 0, R1);
	SetSubMat(P1, 0, 3, t1);
	SetSubMat(P2, 0, 0, R1);
	SetSubMat(P2, 0, 3, t2);
	SetSubMat(P3, 0, 0, R2);
	SetSubMat(P3, 0, 3, t1);
	SetSubMat(P4, 0, 0, R2);
	SetSubMat(P4, 0, 3, t2);
	if (cvDet(R1) < 0)
	{
		ScalarMul(P1, -1, P1);		
		ScalarMul(P2, -1, P2);		
	}

	if (cvDet(R2) < 0)
	{
		ScalarMul(P3, -1, P3);		
		ScalarMul(P4, -1, P4);		
	}
	CvMat *X1 = cvCreateMat(x1->rows, 3, CV_32FC1);
	LinearTriangulation(x1, P0, x2, P1, X1);
	CvMat *X2 = cvCreateMat(x1->rows, 3, CV_32FC1);;
	LinearTriangulation(x1, P0, x2, P2, X2);
	CvMat *X3 = cvCreateMat(x1->rows, 3, CV_32FC1);;
	LinearTriangulation(x1, P0, x2, P3, X3);
	CvMat *X4 = cvCreateMat(x1->rows, 3, CV_32FC1);;
	LinearTriangulation(x1, P0, x2, P4, X4);

	int x1neg = 0, x2neg = 0, x3neg = 0, x4neg = 0;
	CvMat *H1 = cvCreateMat(4, 4, CV_32FC1);	CvMat *invH1 = cvCreateMat(4, 4, CV_32FC1);		CvMat *HX1 = cvCreateMat(X1->rows, X1->cols, CV_32FC1);
	cvSetIdentity(H1);
	SetSubMat(H1, 0, 0, P1);
	cvInvert(H1, invH1);
	Pxx_inhomo(H1, X1, HX1);

	CvMat *H2 = cvCreateMat(4, 4, CV_32FC1);	CvMat *invH2 = cvCreateMat(4, 4, CV_32FC1);		CvMat *HX2 = cvCreateMat(X1->rows, X1->cols, CV_32FC1);
	cvSetIdentity(H2);
	SetSubMat(H2, 0, 0, P2);
	cvInvert(H2, invH2);
	Pxx_inhomo(H2, X2, HX2);
	CvMat *H3 = cvCreateMat(4, 4, CV_32FC1);	CvMat *invH3 = cvCreateMat(4, 4, CV_32FC1);		CvMat *HX3 = cvCreateMat(X1->rows, X1->cols, CV_32FC1);
	cvSetIdentity(H3);
	SetSubMat(H3, 0, 0, P3);
	cvInvert(H3, invH3);
	Pxx_inhomo(H3, X3, HX3);
	CvMat *H4 = cvCreateMat(4, 4, CV_32FC1);	CvMat *invH4 = cvCreateMat(4, 4, CV_32FC1);		CvMat *HX4 = cvCreateMat(X1->rows, X1->cols, CV_32FC1);
	cvSetIdentity(H4);
	SetSubMat(H4, 0, 0, P4);
	cvInvert(H4, invH4);
	Pxx_inhomo(H4, X4, HX4);

	for (int ix = 0; ix < x1->rows; ix++)
	{
		if ((cvGetReal2D(X1, ix, 2)<0) || (cvGetReal2D(HX1, ix, 2)<0))
			x1neg++;
		if ((cvGetReal2D(X2, ix, 2)<0) || (cvGetReal2D(HX2, ix, 2)<0))
			x2neg++;
		if ((cvGetReal2D(X3, ix, 2)<0) || (cvGetReal2D(HX3, ix, 2)<0))
			x3neg++;
		if ((cvGetReal2D(X4, ix, 2)<0) || (cvGetReal2D(HX4, ix, 2)<0))
			x4neg++;
	}

	CvMat *temp34 = cvCreateMat(3, 4, CV_32FC1);
	if ((x1neg <= x2neg) && (x1neg <= x3neg) && (x1neg <= x4neg))
		SetSubMat(P, 0, 0, P1);
	else if ((x2neg <= x1neg) && (x2neg <= x3neg) && (x2neg <= x4neg))
		SetSubMat(P, 0, 0, P2);
	else if ((x3neg <= x1neg) && (x3neg <= x2neg) && (x3neg <= x4neg))
		SetSubMat(P, 0, 0, P3);
	else
		SetSubMat(P, 0, 0, P4);

	//cout << x1neg << " " << x2neg << " " << " " << x3neg << " " << x4neg << endl;
	cvReleaseMat(&W);
	cvReleaseMat(&U);
	cvReleaseMat(&D);
	cvReleaseMat(&Vt);
	cvReleaseMat(&Wt);
	cvReleaseMat(&P0);
	cvReleaseMat(&P1);
	cvReleaseMat(&P2);
	cvReleaseMat(&P3);
	cvReleaseMat(&P4);
	cvReleaseMat(&R1);
	cvReleaseMat(&R2);
	cvReleaseMat(&t1);
	cvReleaseMat(&t2);
	cvReleaseMat(&temp33);
	cvReleaseMat(&temp34);
	cvReleaseMat(&H1);
	cvReleaseMat(&invH1);
	cvReleaseMat(&H2);
	cvReleaseMat(&invH2);
	cvReleaseMat(&H3);
	cvReleaseMat(&invH3);
	cvReleaseMat(&H4);
	cvReleaseMat(&invH4);
	cvReleaseMat(&X1);
	cvReleaseMat(&X2);
	cvReleaseMat(&X3);
	cvReleaseMat(&X4);

	cvReleaseMat(&HX1);
	cvReleaseMat(&HX2);
	cvReleaseMat(&HX3);
	cvReleaseMat(&HX4);
}

void LinearTriangulation(CvMat *x1, CvMat *P1, CvMat *x2, CvMat *P2, CvMat *X)
{
	for (int ix = 0; ix < x1->rows; ix++)
	{
		CvMat *A = cvCreateMat(4, 4, CV_32FC1);
		CvMat *A1 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *A2 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *A3 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *A4 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *P1_1 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *P1_2 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *P1_3 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *P2_1 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *P2_2 = cvCreateMat(1, 4, CV_32FC1);
		CvMat *P2_3 = cvCreateMat(1, 4, CV_32FC1);

		CvMat *temp14_1 = cvCreateMat(1, 4, CV_32FC1);

		GetSubMatRowwise(P1, 0, 0, P1_1);
		GetSubMatRowwise(P1, 1, 1, P1_2);
		GetSubMatRowwise(P1, 2, 2, P1_3);
		GetSubMatRowwise(P2, 0, 0, P2_1);
		GetSubMatRowwise(P2, 1, 1, P2_2);
		GetSubMatRowwise(P2, 2, 2, P2_3);

		ScalarMul(P1_3, cvGetReal2D(x1, ix, 0), temp14_1);
		cvSub(temp14_1, P1_1, A1);

		ScalarMul(P1_3, cvGetReal2D(x1, ix, 1), temp14_1);
		cvSub(temp14_1, P1_2, A2);

		ScalarMul(P2_3, cvGetReal2D(x2, ix, 0), temp14_1);
		cvSub(temp14_1, P2_1, A3);
		ScalarMul(P2_3, cvGetReal2D(x2, ix, 1), temp14_1);
		cvSub(temp14_1, P2_2, A4);
		SetSubMat(A, 0, 0, A1);
		SetSubMat(A, 1, 0, A2);
		SetSubMat(A, 2, 0, A3);
		SetSubMat(A, 3, 0, A4);

		CvMat *x = cvCreateMat(A->cols, 1, CV_32FC1);
		LS_homogeneous(A, x);
		double v = cvGetReal2D(x, 3, 0);
		cvSetReal2D(X, ix, 0, cvGetReal2D(x, 0, 0)/v);
		cvSetReal2D(X, ix, 1, cvGetReal2D(x, 1, 0)/v);
		cvSetReal2D(X, ix, 2, cvGetReal2D(x, 2, 0)/v);

		cvReleaseMat(&A);
		cvReleaseMat(&A1);
		cvReleaseMat(&A2);
		cvReleaseMat(&A3);
		cvReleaseMat(&A4);
		cvReleaseMat(&P1_1);
		cvReleaseMat(&P1_2);
		cvReleaseMat(&P1_3);
		cvReleaseMat(&P2_1);
		cvReleaseMat(&P2_2);
		cvReleaseMat(&P2_3);
		cvReleaseMat(&temp14_1);
		cvReleaseMat(&x);
	}
}


int BilinearCameraPoseEstimation_OPENCV_mem_fast(vector<Feature> &vFeature, int initialFrame1, int initialFrame2, int max_nFrames, vector<Camera> vCamera, CvMat *P, CvMat *X)
{
	PrintAlgorithm("Bilinear Camera Pose Estimation");
	vector<int> visibleFeatureID;
	vector<vector<double> > cx1_vec, cx2_vec, nx1_vec, nx2_vec;

	VisibleIntersection_mem(vFeature, initialFrame1, initialFrame2, cx1_vec, cx2_vec, visibleFeatureID);
	CvMat *cx1 = cvCreateMat(cx1_vec.size(), 2, CV_32FC1);
	CvMat *cx2 = cvCreateMat(cx2_vec.size(), 2, CV_32FC1);
	SetCvMatFromVectors(cx1_vec, cx1);
	SetCvMatFromVectors(cx2_vec, cx2);

	assert(visibleFeatureID.size() > 7);
	CvMat *F = cvCreateMat(3,3,CV_32FC1);
	vector<int> vInlierID;
	CvMat *status = cvCreateMat(1,cx1->rows,CV_8UC1);
	int n = cvFindFundamentalMat(cx1, cx2, F, CV_FM_LMEDS , 1, 0.99, status);
	PrintMat(F, "Fundamental Matrix");
	//for (int i = 0; i < cx1->rows; i++)
	//{
	//	if (cvGetReal2D(status, 0, i) == 1)
	//	{
	//		vInlierID.push_back(visibleFeatureID[i]);
	//	}
	//}

	cout << n << endl;

	vector<int> vCX_indx;
	for (int i = 0; i < cx1->rows; i++)
	{
		CvMat *xM2 = cvCreateMat(1,3,CV_32FC1);
		CvMat *xM1 = cvCreateMat(3,1,CV_32FC1);
		CvMat *s = cvCreateMat(1,1, CV_32FC1);
		cvSetReal2D(xM2, 0, 0, cvGetReal2D(cx2, i, 0));
		cvSetReal2D(xM2, 0, 1, cvGetReal2D(cx2, i, 1));
		cvSetReal2D(xM2, 0, 2, 1);
		cvSetReal2D(xM1, 0, 0, cvGetReal2D(cx1, i, 0));
		cvSetReal2D(xM1, 1, 0, cvGetReal2D(cx1, i, 1));
		cvSetReal2D(xM1, 2, 0, 1);
		cvMatMul(xM2, F, xM2);
		cvMatMul(xM2, xM1, s);			

		double l1 = cvGetReal2D(xM2, 0, 0);
		double l2 = cvGetReal2D(xM2, 0, 1);
		double l3 = cvGetReal2D(xM2, 0, 2);

		double dist = abs(cvGetReal2D(s, 0, 0))/sqrt(l1*l1+l2*l2);

		if (dist < 5)
		{
			vInlierID.push_back(visibleFeatureID[i]);
			vCX_indx.push_back(i);
		}

		cvReleaseMat(&xM2);
		cvReleaseMat(&xM1);
		cvReleaseMat(&s);
		//if (cvGetReal2D(status, 0, i) == 1)
		//{
		//	cvSetReal2D(tempCx1, temprow, 0, cvGetReal2D(cx1, i, 0));
		//	cvSetReal2D(tempCx1, temprow, 1, cvGetReal2D(cx1, i, 1));
		//	cvSetReal2D(tempCx2, temprow, 0, cvGetReal2D(cx2, i, 0));
		//	cvSetReal2D(tempCx2, temprow, 1, cvGetReal2D(cx2, i, 1));
		//	temprow++;
		//}
	}

	visibleFeatureID = vInlierID;
	CvMat *tempCx1 = cvCreateMat(vCX_indx.size(), 2, CV_32FC1);
	CvMat *tempCx2 = cvCreateMat(vCX_indx.size(), 2, CV_32FC1);
	for (int iInlier = 0; iInlier < vInlierID.size(); iInlier++)
	{
		cvSetReal2D(tempCx1, iInlier, 0, cvGetReal2D(cx1, vCX_indx[iInlier], 0));
		cvSetReal2D(tempCx1, iInlier, 1, cvGetReal2D(cx1, vCX_indx[iInlier], 1));
		cvSetReal2D(tempCx2, iInlier, 0, cvGetReal2D(cx2, vCX_indx[iInlier], 0));
		cvSetReal2D(tempCx2, iInlier, 1, cvGetReal2D(cx2, vCX_indx[iInlier], 1));
	}

	cvReleaseMat(&cx1);
	cvReleaseMat(&cx2);
	cx1 = cvCloneMat(tempCx1);
	cx2 = cvCloneMat(tempCx2);
	cvReleaseMat(&status);
	cvReleaseMat(&tempCx1);
	cvReleaseMat(&tempCx2);

	CvMat *E = cvCreateMat(3, 3, CV_32FC1);
	CvMat *temp33 = cvCreateMat(3, 3, CV_32FC1);
	CvMat *temp34 = cvCreateMat(3, 4, CV_32FC1);
	//CvMat *K1 = cvCreateMat(3,3,CV_32FC1);
	//CvMat *K2 = cvCreateMat(3,3,CV_32FC1);
	int camera1 = (int)((double)initialFrame1/max_nFrames);
	int camera2 = (int)((double)initialFrame2/max_nFrames);
	vector<int> ::const_iterator it1 = find(vCamera[camera1].vTakenFrame.begin(), vCamera[camera1].vTakenFrame.end(), initialFrame1%max_nFrames);
	vector<int> ::const_iterator it2 = find(vCamera[camera2].vTakenFrame.begin(), vCamera[camera2].vTakenFrame.end(), initialFrame2%max_nFrames);
	int idx1 = (int) (it1 - vCamera[camera1].vTakenFrame.begin());
	int idx2 = (int) (it2 - vCamera[camera2].vTakenFrame.begin());
	CvMat *K1 = cvCloneMat(vCamera[camera1].vK[idx1]);
	CvMat *K2 = cvCloneMat(vCamera[camera2].vK[idx2]);

	cvTranspose(K2, temp33);
	cvMatMul(temp33, F, temp33);
	cvMatMul(temp33, K1, E);

	CvMat *invK1 = cvCreateMat(3,3,CV_32FC1);
	CvMat *invK2 = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K1, invK1);
	cvInvert(K2, invK2);
	CvMat *nx1 = cvCreateMat(cx1->rows, cx1->cols, CV_32FC1);
	CvMat *nx2 = cvCreateMat(cx2->rows, cx2->cols, CV_32FC1);
	Pxx_inhomo(invK1, cx1, nx1);
	Pxx_inhomo(invK2, cx2, nx2);

	GetExtrinsicParameterFromE(E, nx1, nx2, P);
	CvMat *P0 = cvCreateMat(3, 4, CV_32FC1);
	cvSetIdentity(P0);
	
	CvMat *cX = cvCreateMat(nx1->rows, 3, CV_32FC1);

	PrintMat(P);
	cvMatMul(K1, P0, P0);
	cvMatMul(K2, P, P);

	PrintMat(K1);
	PrintMat(K2);
	//LinearTriangulation(nx1, P0, nx2, P, cX);
	LinearTriangulation(cx1, P0, cx2, P, cX);
	vector<int> vI, vII;
	for (int i = 0; i < cX->rows; i++)
	{
		CvMat *X3d = cvCreateMat(4,1,CV_32FC1);
		cvSetReal2D(X3d, 0, 0, cvGetReal2D(cX, i, 0));
		cvSetReal2D(X3d, 1, 0, cvGetReal2D(cX, i, 1));
		cvSetReal2D(X3d, 2, 0, cvGetReal2D(cX, i, 2));
		cvSetReal2D(X3d, 3, 0, 1);
		CvMat *x11 = cvCreateMat(3,1,CV_32FC1);
		CvMat *x21 = cvCreateMat(3,1,CV_32FC1);
		cvMatMul(P0, X3d, x11);
		cvMatMul(P, X3d, x21);

		//cout << cvGetReal2D(cx1, i, 0) << " " << cvGetReal2D(x11, 0, 0) / cvGetReal2D(x11, 2, 0) << endl;

		if ((cvGetReal2D(x11, 2, 0)>0) && (cvGetReal2D(x21, 2, 0)>0))
		{
			vI.push_back(visibleFeatureID[i]);
			vII.push_back(i);
		}
		cvReleaseMat(&x11);
		cvReleaseMat(&x21);
		cvReleaseMat(&X3d);
	}
	CvMat *X_in = cvCreateMat(vI.size(), 3, CV_32FC1);
	for (int i = 0; i < vI.size(); i++)
	{
		cvSetReal2D(X_in, i, 0, cvGetReal2D(cX, vII[i], 0));
		cvSetReal2D(X_in, i, 1, cvGetReal2D(cX, vII[i], 1));
		cvSetReal2D(X_in, i, 2, cvGetReal2D(cX, vII[i], 2));
	}
	visibleFeatureID = vI;

	//PrintMat(cX);
	cvSetZero(X);
	SetIndexedMatRowwise(X, visibleFeatureID, X_in);
	cvReleaseMat(&X_in);

	
	for (int i = 0; i < visibleFeatureID.size(); i++)
	{
		vFeature[visibleFeatureID[i]].isRegistered = true;
	}

	cvReleaseMat(&F);
	cvReleaseMat(&E);
	cvReleaseMat(&temp33);
	cvReleaseMat(&temp34);
	cvReleaseMat(&K1);
	cvReleaseMat(&K2);
	cvReleaseMat(&invK1);
	cvReleaseMat(&invK2);
	cvReleaseMat(&P0);	
	cvReleaseMat(&cx1);
	cvReleaseMat(&cx2);
	cvReleaseMat(&nx1);
	cvReleaseMat(&nx2);
	cvReleaseMat(&cX);
	return vInlierID.size();
}

int VisibleIntersection23_Simple_fast(vector<Feature> &vFeature, int frame1)
{
	int count = 0;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (!vFeature[iFeature].isRegistered)
			continue;
		vector<int>::iterator it1 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(),frame1);
		if (it1 == vFeature[iFeature].vFrame.end())
			continue;
		count++;
	}
	return count;
}

int DLT_ExtrinsicCameraParamEstimationWRansac_EPNP_mem(CvMat *X, CvMat *x, CvMat *K, CvMat *P, double ransacThreshold, int ransacMaxIter)
{
	int min_set = 4;
	if (X->rows < min_set)
		return 0;

	/////////////////////////////////////////////////////////////////
	// Ransac
	vector<int> vInlierIndex, vOutlierIndex;
	vInlierIndex.clear();
	vOutlierIndex.clear();

	vector<int> vInlier, vOutlier;
	int maxInlier = 0;

	CvMat *X_homoT = cvCreateMat(4, X->rows, CV_32FC1);
	CvMat *X_homo = cvCreateMat(X->rows, 4, CV_32FC1);
	CvMat *x_homoT = cvCreateMat(3, x->rows, CV_32FC1);
	CvMat *x_homo = cvCreateMat(x->rows, 3, CV_32FC1);
	Inhomo2Homo(X, X_homo);
	cvTranspose(X_homo, X_homoT);
	Inhomo2Homo(x, x_homo);
	cvTranspose(x_homo, x_homoT);

	CvMat *randx = cvCreateMat(min_set, 2, CV_32FC1);
	CvMat *randX = cvCreateMat(min_set, 3, CV_32FC1);
	CvMat *randP = cvCreateMat(3,4,CV_32FC1);
	int *randIdx = (int *) malloc(min_set * sizeof(int));

	CvMat *reproj = cvCreateMat(3,1,CV_32FC1);
	CvMat *homo_X = cvCreateMat(4,1,CV_32FC1);
	for (int iRansacIter = 0; iRansacIter < ransacMaxIter; iRansacIter++)
	{		
		for (int iIdx = 0; iIdx < min_set; iIdx++)
			randIdx[iIdx] = rand()%X->rows;

		for (int iIdx = 0; iIdx < min_set; iIdx++)
		{
			cvSetReal2D(randx, iIdx, 0, cvGetReal2D(x, randIdx[iIdx], 0));
			cvSetReal2D(randx, iIdx, 1, cvGetReal2D(x, randIdx[iIdx], 1));
			cvSetReal2D(randX, iIdx, 0, cvGetReal2D(X, randIdx[iIdx], 0));
			cvSetReal2D(randX, iIdx, 1, cvGetReal2D(X, randIdx[iIdx], 1));
			cvSetReal2D(randX, iIdx, 2, cvGetReal2D(X, randIdx[iIdx], 2));
		}
		EPNP_ExtrinsicCameraParamEstimation(randX, randx, K, randP);

		vInlier.clear();
		vOutlier.clear();
		for (int ip = 0; ip < X->rows; ip++)
		{
			cvSetReal2D(homo_X, 0, 0, cvGetReal2D(X, ip, 0));
			cvSetReal2D(homo_X, 1, 0, cvGetReal2D(X, ip, 1));
			cvSetReal2D(homo_X, 2, 0, cvGetReal2D(X, ip, 2));
			cvSetReal2D(homo_X, 3, 0, 1);

			cvMatMul(randP, homo_X, reproj);
			double u = cvGetReal2D(reproj, 0, 0)/cvGetReal2D(reproj, 2, 0);
			double v = cvGetReal2D(reproj, 1, 0)/cvGetReal2D(reproj, 2, 0);
			double dist = sqrt((u-cvGetReal2D(x, ip, 0))*(u-cvGetReal2D(x, ip, 0))+(v-cvGetReal2D(x, ip, 1))*(v-cvGetReal2D(x, ip, 1)));
			if (dist < ransacThreshold)
			{
				vInlier.push_back(ip);
			}
			else
			{
				vOutlier.push_back(ip);
			}

		}
		

		if (vInlier.size() > maxInlier)
		{
			maxInlier = vInlier.size();
			SetSubMat(P, 0, 0, randP);
			vInlierIndex = vInlier;
			vOutlierIndex = vOutlier;
		}

		if (vInlier.size() > X->rows * 0.8)
		{
			break;
		}
	}
	CvMat *Xin = cvCreateMat(vInlierIndex.size(), 3, CV_32FC1);
	CvMat *xin = cvCreateMat(vInlierIndex.size(), 2, CV_32FC1);
	for (int iInlier = 0; iInlier < vInlierIndex.size(); iInlier++)
	{
		cvSetReal2D(Xin, iInlier, 0, cvGetReal2D(X, vInlierIndex[iInlier], 0));
		cvSetReal2D(Xin, iInlier, 1, cvGetReal2D(X, vInlierIndex[iInlier], 1));
		cvSetReal2D(Xin, iInlier, 2, cvGetReal2D(X, vInlierIndex[iInlier], 2));

		cvSetReal2D(xin, iInlier, 0, cvGetReal2D(x, vInlierIndex[iInlier], 0));
		cvSetReal2D(xin, iInlier, 1, cvGetReal2D(x, vInlierIndex[iInlier], 1));
	}
	//EPNP_ExtrinsicCameraParamEstimation(Xin, xin, K, P);

	cvReleaseMat(&Xin);
	cvReleaseMat(&xin);
	cvReleaseMat(&reproj);
	cvReleaseMat(&homo_X);
	free(randIdx);
	cvReleaseMat(&randx);
	cvReleaseMat(&randX);
	cvReleaseMat(&randP);

	cvReleaseMat(&X_homoT);
	cvReleaseMat(&x_homo);
	cvReleaseMat(&x_homoT);
	cvReleaseMat(&X_homo);
	if (vInlierIndex.size() < 30)
		return 0;
	cout << "Number of features ePnP: " << vInlierIndex.size() << endl;
	return vInlierIndex.size();
}

int EPNP_ExtrinsicCameraParamEstimation(CvMat *X, CvMat *x, CvMat *K, CvMat *P)
{
	epnp PnP;

	PnP.set_internal_parameters(cvGetReal2D(K, 0, 2), cvGetReal2D(K, 1, 2), cvGetReal2D(K, 0, 0), cvGetReal2D(K, 1, 1));
	PnP.set_maximum_number_of_correspondences(X->rows);
	PnP.reset_correspondences();
	for(int i = 0; i < X->rows; i++) {
		PnP.add_correspondence(cvGetReal2D(X, i, 0), cvGetReal2D(X, i, 1), cvGetReal2D(X, i, 2), cvGetReal2D(x, i, 0), cvGetReal2D(x, i, 1));
	}

	double R_est[3][3], t_est[3];
	double err2 = PnP.compute_pose(R_est, t_est);

	cvSetReal2D(P, 0, 3, t_est[0]);
	cvSetReal2D(P, 1, 3, t_est[1]);
	cvSetReal2D(P, 2, 3, t_est[2]);

	cvSetReal2D(P, 0, 0, R_est[0][0]);		cvSetReal2D(P, 0, 1, R_est[0][1]);		cvSetReal2D(P, 0, 2, R_est[0][2]);
	cvSetReal2D(P, 1, 0, R_est[1][0]);		cvSetReal2D(P, 1, 1, R_est[1][1]);		cvSetReal2D(P, 1, 2, R_est[1][2]);
	cvSetReal2D(P, 2, 0, R_est[2][0]);		cvSetReal2D(P, 2, 1, R_est[2][1]);		cvSetReal2D(P, 2, 2, R_est[2][2]);
	cvMatMul(K, P, P);

	return 1;
}
