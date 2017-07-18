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
#include "DataUtility.h"
//#include "MultiviewGeometryUtility.h"
#include "MathUtility.h"

#define DLT_RANSAC_THRESHOLD 12e-0
#define DLT_RANSAC_MAX_ITER 3e+4
#define F_ESTIMATION_THRESHOLD_INIT 1e-1
#define F_ESTIMATION_MAX_ITER_INIT 5e+3
#define F_ESTIMATION_THRESHOLD 1e+1
#define F_ESTIMATION_MAX_ITER 1e+3
#define MAX_FRAMES 2e+3

//#define FILE_PATH "D:/03 Works/Data/SocialDynamics/temp/pingpong_ref/"
//#define FILE_PATH "I:/EDSH_demo/"
//#define MIDDLE_FOLDER "reconstruction/"
#define FILE_PATH ""
#define MIDDLE_FOLDER "reconstruction/"

struct InitialFrame
{
	int init1, init2;
	double dist;
	double portion;
	int nCorr;
};

void VisibleIntersection_mem(vector<Feature> &vFeature, int frame1, int frame2, vector<vector<double> > &cx1, vector<vector<double> > &cx2, vector<int> &visibleFeatureID)
{
	vector<double> x1, y1, x2, y2;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
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


bool InitialFrameComparison (InitialFrame init1, InitialFrame init2)
{
	if (init1.portion == init2.portion)
	{
		return (init1.dist > init2.dist);
	}
	else
		return (init1.portion < init2.portion);
}

void SetCvMatFromVectors(vector<vector<double> > x, CvMat *X)
{
	for (int i = 0; i < x.size(); i++)
	{
		for (int j = 0; j < x[i].size(); j++)
			cvSetReal2D(X, i, j, x[i][j]);
	}
}


int main ( int argc, char * argv[] )
{
	srand(time(NULL));
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Load data
	string path = FILE_PATH;
	string middleFolder = MIDDLE_FOLDER;
	string initialFile = middleFolder + "initial_frame.txt";
	string initialFile_list = middleFolder + "initial_frame_list.txt";
	string measurementFile = middleFolder + "stitchedmeasurement_static.txt";
	string measurementFile_save = middleFolder + "stitchedmeasurement_static_save.txt";
	string cameraFile = middleFolder + "camera.txt";
	string structureFile = middleFolder + "structure.txt";

	vector<Feature> vFeature;

	///////////////////////////////////////////////
	// Load correspondence file
	int max_frame;
	LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast_InitialFrame(path+measurementFile, vFeature, max_frame);

	SaveStaticMeasurementData_NoCalibration_RGB_LOWES_fast_InitialFrame(path+measurementFile, vFeature, max_frame);

	vector<int> vi11, vi12;
	for (int iInit1 = 0; iInit1 < max_frame-1; iInit1++)
	{
		for (int iInit2 = iInit1+1; iInit2 < max_frame; iInit2++)
		{
			vi11.push_back(iInit1);
			vi12.push_back(iInit2);
		}
	}

	vector<InitialFrame> vInit;
	vector<bool> isIn;
	vInit.resize(vi11.size());
	isIn.resize(vi11.size());
	#pragma omp parallel for
	for (int ivi = 0; ivi < vi11.size(); ivi++)
	{
		int iInit1 = vi11[ivi];
		int iInit2 = vi12[ivi];
		vector<int> visibleFeatureID;
		vector<vector<double> > cx1_vec, cx2_vec, nx1_vec, nx2_vec;

		VisibleIntersection_mem(vFeature, iInit1, iInit2, cx1_vec, cx2_vec, visibleFeatureID);

		if (visibleFeatureID.size() < 300)
 			continue;

		CvMat *cx1 = cvCreateMat(cx1_vec.size(), 2, CV_32FC1);
		CvMat *cx2 = cvCreateMat(cx2_vec.size(), 2, CV_32FC1);
		SetCvMatFromVectors(cx1_vec, cx1);
		SetCvMatFromVectors(cx2_vec, cx2);

		CvMat *H = cvCreateMat(3,3,CV_32FC1);
		CvMat *F = cvCreateMat(3,3,CV_32FC1);
		vector<int> vInlierID;
		CvMat *status = cvCreateMat(1, cx1->rows, CV_8UC1);

		int n = cvFindFundamentalMat(cx1, cx2, F, CV_FM_RANSAC , 1, 0.99, status);
		for (int i = 0; i < cx1->rows; i++)
		{
			if (cvGetReal2D(status, 0, i) == 1)
			{
				vInlierID.push_back(i);
			}
		}

		if (vInlierID.size() < 100)
		{
			cvReleaseMat(&cx1);
			cvReleaseMat(&cx2);
			cvReleaseMat(&status);
			cvReleaseMat(&F);
			cvReleaseMat(&H);
			continue;
		}

		CvMat *cx1_h = cvCreateMat(vInlierID.size(), 2, CV_32FC1);
		CvMat *cx2_h = cvCreateMat(vInlierID.size(), 2, CV_32FC1);

		for (int iInlier = 0; iInlier < vInlierID.size(); iInlier++)
		{
			cvSetReal2D(cx1_h, iInlier, 0, cvGetReal2D(cx1, vInlierID[iInlier], 0));
			cvSetReal2D(cx1_h, iInlier, 1, cvGetReal2D(cx1, vInlierID[iInlier], 1));

			cvSetReal2D(cx2_h, iInlier, 0, cvGetReal2D(cx2, vInlierID[iInlier], 0));
			cvSetReal2D(cx2_h, iInlier, 1, cvGetReal2D(cx2, vInlierID[iInlier], 1));
		}

		n = cvFindHomography(cx1_h, cx2_h, H);

		double H11 = cvGetReal2D(H, 0, 0);
		double H12 = cvGetReal2D(H, 0, 1);
		double H13 = cvGetReal2D(H, 0, 2);

		double H21 = cvGetReal2D(H, 1, 0);
		double H22 = cvGetReal2D(H, 1, 1);
		double H23 = cvGetReal2D(H, 1, 2);

		double H31 = cvGetReal2D(H, 2, 0);
		double H32 = cvGetReal2D(H, 2, 1);
		double H33 = cvGetReal2D(H, 2, 2);

		int count=0;
		double ave_dist = 0;
		for (int i = 0; i < cx1->rows; i++)
		{
			double x1 = cvGetReal2D(cx1, i, 0);
			double y1 = cvGetReal2D(cx1, i, 1);
			double x2 = cvGetReal2D(cx2, i, 0);
			double y2 = cvGetReal2D(cx2, i, 1);
			double u = H11*x1+H12*y1+H13;
			double v = H21*x1+H22*y1+H23;
			double w = H31*x1+H32*y1+H33;
			u = u/w;
			v = v/w;
			double dist = sqrt((u-x2)*(u-x2)+(v-y2)*(v-y2));
			ave_dist += dist;
			if (dist < 5)
				count++;
		}

		if (count==0)
		{
			
			cvReleaseMat(&cx1);
			cvReleaseMat(&cx1_h);
			cvReleaseMat(&cx2);
			cvReleaseMat(&cx2_h);
			cvReleaseMat(&status);
			cvReleaseMat(&F);
			cvReleaseMat(&H);
			continue;
		}

		ave_dist /= cx1->rows;

		cout << iInit1 << " " << iInit2 << " " << ((double)count)/cx1->rows << " " << ave_dist << endl;

		InitialFrame initFrame;
		initFrame.init1 = iInit1;
		initFrame.init2 = iInit2;
		initFrame.dist = ave_dist;
		initFrame.nCorr = cx1->rows;
		initFrame.portion = ((double)count)/cx1->rows;
		vInit[ivi] = initFrame;
		isIn[ivi] = true;

		cvReleaseMat(&cx1);
		cvReleaseMat(&cx1_h);
		cvReleaseMat(&cx2);
		cvReleaseMat(&cx2_h);
		cvReleaseMat(&status);
		cvReleaseMat(&F);
		cvReleaseMat(&H);
	}

	vector<InitialFrame> vInit_temp;
	for (int i = 0; i < vInit.size(); i++)
	{
		if (isIn[i])
			vInit_temp.push_back(vInit[i]);
	}
	vInit = vInit_temp;

	sort(vInit.begin(), vInit.end(), InitialFrameComparison);
	SaveInitialFrame(path+initialFile, vInit[0].init1, vInit[0].init2, max_frame);

	vector<int> vi1, vi2;
	vector<double> vd, vp;
	vector<int> ncorr;
	for (int i=0; i<vInit.size(); i++)
	{
		vi1.push_back(vInit[i].init1);
		vi2.push_back(vInit[i].init2);
		vd.push_back(vInit[i].dist);
		vp.push_back(vInit[i].portion);
		ncorr.push_back(vInit[i].nCorr);
	}

	SaveInitialFrameList(path+initialFile_list, vi1, vi2, vd, vp, ncorr);
	return 0;
}
