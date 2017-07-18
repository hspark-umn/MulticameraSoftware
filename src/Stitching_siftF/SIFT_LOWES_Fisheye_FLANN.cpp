#include <fstream>
#include <iostream>
#include <string>
#include <omp.h>
#include <boost/filesystem.hpp> 
#include <flann/flann.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <algorithm>
#include "DataUtility.h"
#include "StructDefinition.h"

#define STATIC true
#define STITCHING false
#define ZERO_DISTANCE 1e+0
#define DESCRIPTOR_ZERO_DISTANCE 2e+2
#define FILE_PATH ""

void SIFT_Stitching(string path);
void SIFT_Stitching(string path, int nThread);
void Iterate_SIFT_STATIC(FrameCamera &cFC, vector<FrameCamera> &vCurrentFC, CvMat *K, CvMat *invK, double omega, vector<Feature> &feature, bool display);
void DoSIFT(CvMat *desc1, CvMat *desc2, vector<int> &vPoint1, vector<int> &vPoint2);
void DoSIFT(float *desc1, float *desc2, int nPoint1, int nPoint2, vector<int> &vPoint1, vector<int> &vPoint2);
int GetStaticCorrespondences(vector<Point> x1, vector<Point> x2, vector<bool> &vIsInlier);
void SIFT_Stitching_MP(string path, string input, int nFiles);
void Iterate_SIFT_STATIC_MP(vector<FrameCamera> &vFC, int currentFC, CvMat *K, CvMat *invK, double omega, vector<Feature> &feature_static, bool display);
void Stitching(vector<Feature> &vFeature, vector<Feature> &vStitchedFeature, int,int,int);
void Stitching(vector<Feature> &vStitchedFeature, Feature feature, int &feature_idx, Feature &newFeature,
				int max_nCameras, int max_nFrames);
void SIFT_Stitching_MP_OPENMP(string path, string input, int nFiles);
using namespace std;

bool IsSamePoint(Point p1, Point p2)
{
	if (DistancePixel(p1.x, p1.y, p2.x, p2.y) < ZERO_DISTANCE)
		return true;
	else
		return false;
}

bool IsSamePoint(vector<double> &desc1, vector<double> &desc2)
{
	double norm = 0;
	for (int i = 0; i < 128; i++)
	{
		norm = norm + (desc1[i]-desc2[i])*(desc1[i]-desc2[i]);
	}
	norm = sqrt(norm);

	if (norm < DESCRIPTOR_ZERO_DISTANCE)
		return true;
	else 
		return false;

}

void Undistortion(CvMat *K, CvMat *invK, double omega, vector<double> &vx,  vector<double> &vy)
{
	for (int iPoint = 0; iPoint < vx.size(); iPoint++)
	{
		CvMat *x_homo = cvCreateMat(3,1,CV_32FC1);
		cvSetReal2D(x_homo, 0, 0, vx[iPoint]);
		cvSetReal2D(x_homo, 1, 0, vy[iPoint]);
		cvSetReal2D(x_homo, 2, 0, 1);
		CvMat *x_homo_n = cvCreateMat(3,1,CV_32FC1);
		cvMatMul(invK, x_homo, x_homo_n);
		double x_n, y_n;
		x_n = cvGetReal2D(x_homo_n, 0, 0);
		y_n = cvGetReal2D(x_homo_n, 1, 0);
		double r_d = sqrt(x_n*x_n+y_n*y_n);
		double r_u = tan(r_d*omega)/2/tan(omega/2); 
		double x_u = r_u/r_d*x_n;
		double y_u = r_u/r_d*y_n;
		CvMat *x_undist_n = cvCreateMat(3,1,CV_32FC1);
		cvSetReal2D(x_undist_n, 0, 0, x_u);
		cvSetReal2D(x_undist_n, 1, 0, y_u);
		cvSetReal2D(x_undist_n, 2, 0, 1);
		CvMat *x_undist = cvCreateMat(3,1,CV_32FC1);
		cvMatMul(K, x_undist_n, x_undist);
		vx[iPoint] = cvGetReal2D(x_undist,0,0);
		vy[iPoint] = cvGetReal2D(x_undist,1,0);

		cvReleaseMat(&x_homo);
		cvReleaseMat(&x_homo_n);
		cvReleaseMat(&x_undist_n);
		cvReleaseMat(&x_undist);
	}
}

// Stitching loop

int main ( int argc, char * argv[] )
{
	////////////////////////////////////////////////////////////
	// Filename setting
	string path = FILE_PATH;
	string savepath = path + "reconstruction/";
	string savepath_m = savepath + "measurement/";
	string fileinfofile = path + "FileInfo.TXT";
	string savefile_static = savepath_m + "static_measurement_desc%07d.txt";

	vector<Camera> vCamera;
	LoadFileInfo(fileinfofile, FILE_PATH, vCamera);
	
	//SIFT_Stitching_MP(savepath, savefile_static, vCamera[0].vTakenFrame.size());
	SIFT_Stitching_MP_OPENMP(savepath, savefile_static, vCamera[0].vTakenFrame.size());
	return 0;
} // end main()

void ComputeMeanDescriptor(vector<double> vMean1, vector<double> vMean2, vector<double> &vMean)
{
	for (int i = 0; i < 128; i++)
	{
		int mean1 = (int)floor((double)(vMean1[i]*vMean1[128] + vMean2[i])/(double)(vMean1[128]+1));
		vMean.push_back(mean1);
		//if (mean1 < 0)
		//	int k = 1;
	}
	vMean.push_back(vMean1[128]+1);
}

void SIFT_Stitching_MP(string path, string input, int nFiles)
{
	vector<Feature> vFeature, vTempFeature;
	int nFeatures, nTempFeatures;

	vector<int> vnFrames;
	string savefile = path + "stitchedmeasurement_static.txt";
	string savefile_desc = path + "descriptors.txt";

	int max_nCameras = 1;
	int nCameras = 1;

	for (int i = 0; i < nFiles; i++)
	{
		char temp[1000];
		sprintf(temp, input.c_str(), i);
		string filename = temp;

		LoadMeasurementData_RGB_DESC_Seq(filename, vFeature);
	}
	int max_nFrames = 0;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		for (int iFrame = 0; iFrame < vFeature[iFeature].vFrame.size(); iFrame++)
		{
			if (max_nFrames < vFeature[iFeature].vFrame[iFrame])
				max_nFrames = vFeature[iFeature].vFrame[iFrame];
		}
	}
	max_nFrames++;
	nFeatures = vFeature.size();
	vnFrames.push_back(max_nFrames);

	vector<Feature> vStitchedFeature;
	vector<Point> vPoint_feature;
	vector<Point> vPoint_stitchedFeature;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (iFeature % 1000 == 1)
			cout << iFeature << " / " << vFeature.size() << endl;
		bool isInSet = false;
		for (int iStitchedFeature = 0; iStitchedFeature < vStitchedFeature.size(); iStitchedFeature++)
		{
			if (isInSet)
				break;
			if (vStitchedFeature[iStitchedFeature].vFrame[0] > vFeature[iFeature].vFrame[vFeature[iFeature].vFrame.size()-1])
				continue;
			if (vStitchedFeature[iStitchedFeature].vFrame[vStitchedFeature[iStitchedFeature].vFrame.size()-1] < vFeature[iFeature].vFrame[0])
				continue;

			// Find the same frame
			for (int iFeatureFrame = 0; iFeatureFrame < vFeature[iFeature].vFrame.size(); iFeatureFrame++)
			{
				if ((vFeature[iFeature].vFrame[iFeatureFrame] < vStitchedFeature[iStitchedFeature].vFrame[0])||
					(vFeature[iFeature].vFrame[iFeatureFrame] > vStitchedFeature[iStitchedFeature].vFrame[vStitchedFeature[iStitchedFeature].vFrame.size()-1]))
					continue;

				vector<int>::const_iterator it = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), 
					vFeature[iFeature].vFrame[iFeatureFrame]);
				if (it != vStitchedFeature[iStitchedFeature].vFrame.end())
				{
					int idx = (int) (it - vStitchedFeature[iStitchedFeature].vFrame.begin());
					Point pFeature, pStitchedFeature;
					pFeature.x = vFeature[iFeature].vx[iFeatureFrame];
					pFeature.y = vFeature[iFeature].vy[iFeatureFrame];
					pStitchedFeature.x = vStitchedFeature[iStitchedFeature].vx[idx];
					pStitchedFeature.y = vStitchedFeature[iStitchedFeature].vy[idx];
					//if (IsSamePoint(pFeature, pStitchedFeature, vFeature[iFeature].vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0]))
					if (IsSamePoint(pFeature, pStitchedFeature))
					{
						//if (IsSamePoint(vFeature[iFeature].vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0]))
						{
							Feature fs;
							fs.id = vStitchedFeature[iStitchedFeature].id;
							fs.r = vStitchedFeature[iStitchedFeature].r;
							fs.g = vStitchedFeature[iStitchedFeature].g;
							fs.b = vStitchedFeature[iStitchedFeature].b;
							int temp = 0;
							for (int iTotalFrame = 0; iTotalFrame < max_nCameras*max_nFrames; iTotalFrame++)
							{
								vector<int>::const_iterator it_total1 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(), iTotalFrame);
								vector<int>::const_iterator it_total2 = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), iTotalFrame);
								if ((it_total1 != vFeature[iFeature].vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
								{
									int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
									fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
									fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
									fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
									fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
									fs.vx_dis.push_back(vStitchedFeature[iStitchedFeature].vx_dis[idx_total]);
									fs.vy_dis.push_back(vStitchedFeature[iStitchedFeature].vy_dis[idx_total]);
									vector<double> meanDesc;
									if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
									{
										if (fs.vvDesc.size() == 0)
										{
											meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
										}
										else
										{
											meanDesc = fs.vvDesc[0];
										}
									}
									else
									{
										ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
									}							
									fs.vvDesc.clear();
									fs.vvDesc.push_back(meanDesc);
								}
								else if ((it_total1 != vFeature[iFeature].vFrame.end()) && (it_total2 == vStitchedFeature[iStitchedFeature].vFrame.end()))
								{
									int idx_total = (int) (it_total1 - vFeature[iFeature].vFrame.begin());
									fs.vFrame.push_back(vFeature[iFeature].vFrame[idx_total]);
									fs.vCamera.push_back(vFeature[iFeature].vCamera[idx_total]);
									fs.vx.push_back(vFeature[iFeature].vx[idx_total]);
									fs.vy.push_back(vFeature[iFeature].vy[idx_total]);
									fs.vx_dis.push_back(vFeature[iFeature].vx_dis[idx_total]);
									fs.vy_dis.push_back(vFeature[iFeature].vy_dis[idx_total]);
									//fs.vvDesc.push_back(vFeature[iFeature].vvDesc[idx_total]);
									vector<double> meanDesc;
									if ((fs.vvDesc.size() == 0) || (vFeature[iFeature].vvDesc.size() == 0))
									{
										if (fs.vvDesc.size() == 0)
										{
											meanDesc = vFeature[iFeature].vvDesc[0];
										}
										else
										{
											meanDesc = fs.vvDesc[0];
										}
									}
									else
									{
										ComputeMeanDescriptor(fs.vvDesc[0], vFeature[iFeature].vvDesc[0], meanDesc);
									}							
									fs.vvDesc.clear();
									fs.vvDesc.push_back(meanDesc);
								}
								else if ((it_total1 == vFeature[iFeature].vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
								{
									int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
									fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
									fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
									fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
									fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
									fs.vx_dis.push_back(vStitchedFeature[iStitchedFeature].vx_dis[idx_total]);
									fs.vy_dis.push_back(vStitchedFeature[iStitchedFeature].vy_dis[idx_total]);
									//fs.vvDesc.push_back(vStitchedFeature[iStitchedFeature].vvDesc[idx_total]);
									vector<double> meanDesc;
									if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
									{
										if (fs.vvDesc.size() == 0)
										{
											meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
										}
										else
										{
											meanDesc = fs.vvDesc[0];
										}
									}
									else
									{
										ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
									}							
									fs.vvDesc.clear();
									fs.vvDesc.push_back(meanDesc);
								}
								temp = iTotalFrame;
							}

							//if (fs.vFrame.size() == 0)
							//{
							//	cout << endl << temp << endl;
							//	for (int i =0; i < vStitchedFeature[iStitchedFeature].vFrame.size(); i++)
							//		cout << vStitchedFeature[iStitchedFeature].vFrame[i] << " ";
							//}
							vStitchedFeature[iStitchedFeature] = fs;
							isInSet = true;
						}
					}
					if (isInSet)
						break;
				}
			}
		}
		if (!isInSet)
			vStitchedFeature.push_back(vFeature[iFeature]);
	}

	SaveMeasurementData_RGB(savefile, vStitchedFeature, max_nFrames, FILESAVE_WRITE_MODE);
	SaveMeasurementData_DESC(savefile_desc, vStitchedFeature, max_nFrames, FILESAVE_WRITE_MODE);
	//SaveMeasurementData(savefile, vStitchedFeature, FILESAVE_WRITE_MODE);
	//ResaveMeasurementData(savefile, vnFrames, nCameras);
	return;
}

void SIFT_Stitching_MP_OPENMP(string path, string input, int nFiles)
{
	vector<Feature> vFeature, vTempFeature;
	int nFeatures, nTempFeatures;

	vector<int> vnFrames;
	string savefile = path + "stitchedmeasurement_static.txt";
	string savefile_desc = path + "descriptors.txt";

	int max_nCameras = 1;
	int nCameras = 1;

	for (int i = 0; i < nFiles; i++)
	{
		char temp[1000];
		sprintf(temp, input.c_str(), i);
		string filename = temp;

		LoadMeasurementData_RGB_DESC_Seq(filename, vFeature);
	}
	int max_nFrames = 0;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		for (int iFrame = 0; iFrame < vFeature[iFeature].vFrame.size(); iFrame++)
		{
			if (max_nFrames < vFeature[iFeature].vFrame[iFrame])
				max_nFrames = vFeature[iFeature].vFrame[iFrame];
		}
	}
	max_nFrames++;
	nFeatures = vFeature.size();
	vnFrames.push_back(max_nFrames);

	vector<vector<Feature> > vvFeature;
	//vector<vector<Feature> > vvStitchedFeature;
	//vector<Point> vPoint_feature;
	//vector<Point> vPoint_stitchedFeature;

	int nThread = omp_get_max_threads()-1;
	int nFeature_each = vFeature.size()/nThread;

	int iFeature_num = 0;
	for (int i = 0; i < nThread-1; i++)
	{
		vector<Feature> vFeature1;
		for (int iFeature = 0; iFeature < nFeature_each; iFeature++)
		{
			vFeature1.push_back(vFeature[iFeature_num]);
			iFeature_num++;
		}
		vvFeature.push_back(vFeature1);
	}

	vector<Feature> vFeature1;
	vFeature1.clear();
	for (int i = iFeature_num; i < vFeature.size(); i++)
	{
		vFeature1.push_back(vFeature[i]);
	}
	vvFeature.push_back(vFeature1);
	vFeature.clear();
	vFeature1.clear();

	vector<vector<Feature> > vvStitchedFeature;
	vvStitchedFeature.resize(vvFeature.size());
	int nTotal_feature = 0;
	#pragma omp parallel for 
	for (int iThread = 0; iThread < vvFeature.size(); iThread++)
	{
		vector<Feature> vStitchedFeature1;
		Stitching(vvFeature[iThread], vStitchedFeature1, max_nCameras, max_nFrames,iThread);
		vvStitchedFeature[iThread] = vStitchedFeature1;
	}

	for (int iThread = 0; iThread < nThread; iThread++)
	{
		nTotal_feature += vvStitchedFeature[iThread].size();
	}
	nTotal_feature -= vvStitchedFeature[0].size();

	vector<Feature> vStitchedFeature, vFeature_new;
	vStitchedFeature = vvStitchedFeature[0];
	int count = 0;
	for (int i = 1; i < vvStitchedFeature.size(); i++)
	{
		vector<int> vIdx;
		vector<Feature> vFeature_1;
		vIdx.resize(vvStitchedFeature[i].size());
		vFeature_1.resize(vvStitchedFeature[i].size());
		//vector<vector<Feature> > vvStitchedFeature1;
		//int nTh = omp_get_num_threads();
		//for (int j = 0; j < nTh; j++)
		//{
		//	vvStitchedFeature1.push_back(vStitchedFeature);
		//}
		#pragma omp parallel for 
		for (int iFeature = 0; iFeature < vvStitchedFeature[i].size(); iFeature++)
		{
			//int th_id = omp_get_thread_num();
			int idx = -1;
			Feature fs; 
			Stitching(vStitchedFeature, vvStitchedFeature[i][iFeature], idx, fs, max_nCameras, max_nFrames);
		
			vIdx[iFeature] = idx;
			vFeature_1[iFeature] = fs;
			count++;
			if (count%1000 ==0)
				cout << count << " " << nTotal_feature << endl;;
		}

		//vvStitchedFeature1.clear();

		for (int iIdx = 0; iIdx < vIdx.size(); iIdx++)
		{
			//if (iIdx < 100)
			//	cout << vIdx[iIdx] << " ";
			if (vIdx[iIdx] != -1)
				vStitchedFeature[vIdx[iIdx]] = vFeature_1[iIdx];
			else
				vStitchedFeature.push_back(vFeature_1[iIdx]);
		}
		//break;
	}


	SaveMeasurementData_RGB(savefile, vStitchedFeature, max_nFrames, FILESAVE_WRITE_MODE);
	SaveMeasurementData_DESC(savefile_desc, vStitchedFeature, max_nFrames, FILESAVE_WRITE_MODE);
	//SaveMeasurementData(savefile, vStitchedFeature, FILESAVE_WRITE_MODE);
	//ResaveMeasurementData(savefile, vnFrames, nCameras);
	return;
}

void Stitching(vector<Feature> &vFeature, vector<Feature> &vStitchedFeature,
				int max_nCameras, int max_nFrames, int iThread)
{
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (iFeature % 1000 == 1)
			cout << iThread << " / " << iFeature << " / " << vFeature.size() << endl;
		bool isInSet = false;
		for (int iStitchedFeature = 0; iStitchedFeature < vStitchedFeature.size(); iStitchedFeature++)
		{
			if (isInSet)
				break;
			if (vStitchedFeature[iStitchedFeature].vFrame[0] > vFeature[iFeature].vFrame[vFeature[iFeature].vFrame.size()-1])
				continue;
			if (vStitchedFeature[iStitchedFeature].vFrame[vStitchedFeature[iStitchedFeature].vFrame.size()-1] < vFeature[iFeature].vFrame[0])
				continue;

			// Find the same frame
			for (int iFeatureFrame = 0; iFeatureFrame < vFeature[iFeature].vFrame.size(); iFeatureFrame++)
			{
				if ((vFeature[iFeature].vFrame[iFeatureFrame] < vStitchedFeature[iStitchedFeature].vFrame[0])||
					(vFeature[iFeature].vFrame[iFeatureFrame] > vStitchedFeature[iStitchedFeature].vFrame[vStitchedFeature[iStitchedFeature].vFrame.size()-1]))
					continue;

				vector<int>::const_iterator it = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), 
					vFeature[iFeature].vFrame[iFeatureFrame]);
				if (it != vStitchedFeature[iStitchedFeature].vFrame.end())
				{
					int idx = (int) (it - vStitchedFeature[iStitchedFeature].vFrame.begin());
					Point pFeature, pStitchedFeature;
					pFeature.x = vFeature[iFeature].vx[iFeatureFrame];
					pFeature.y = vFeature[iFeature].vy[iFeatureFrame];
					pStitchedFeature.x = vStitchedFeature[iStitchedFeature].vx[idx];
					pStitchedFeature.y = vStitchedFeature[iStitchedFeature].vy[idx];
					//if (IsSamePoint(pFeature, pStitchedFeature, vFeature[iFeature].vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0]))
					if (IsSamePoint(pFeature, pStitchedFeature))
					{
						//if (IsSamePoint(vFeature[iFeature].vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0]))
						{
							Feature fs;
							fs.id = vStitchedFeature[iStitchedFeature].id;
							fs.r = vStitchedFeature[iStitchedFeature].r;
							fs.g = vStitchedFeature[iStitchedFeature].g;
							fs.b = vStitchedFeature[iStitchedFeature].b;
							int temp = 0;
							for (int iTotalFrame = 0; iTotalFrame < max_nCameras*max_nFrames; iTotalFrame++)
							{
								vector<int>::const_iterator it_total1 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(), iTotalFrame);
								vector<int>::const_iterator it_total2 = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), iTotalFrame);
								if ((it_total1 != vFeature[iFeature].vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
								{
									int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
									fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
									fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
									fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
									fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
									fs.vx_dis.push_back(vStitchedFeature[iStitchedFeature].vx_dis[idx_total]);
									fs.vy_dis.push_back(vStitchedFeature[iStitchedFeature].vy_dis[idx_total]);
									vector<double> meanDesc;
									if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
									{
										if (fs.vvDesc.size() == 0)
										{
											meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
										}
										else
										{
											meanDesc = fs.vvDesc[0];
										}
									}
									else
									{
										ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
									}							
									fs.vvDesc.clear();
									fs.vvDesc.push_back(meanDesc);
								}
								else if ((it_total1 != vFeature[iFeature].vFrame.end()) && (it_total2 == vStitchedFeature[iStitchedFeature].vFrame.end()))
								{
									int idx_total = (int) (it_total1 - vFeature[iFeature].vFrame.begin());
									fs.vFrame.push_back(vFeature[iFeature].vFrame[idx_total]);
									fs.vCamera.push_back(vFeature[iFeature].vCamera[idx_total]);
									fs.vx.push_back(vFeature[iFeature].vx[idx_total]);
									fs.vy.push_back(vFeature[iFeature].vy[idx_total]);
									fs.vx_dis.push_back(vFeature[iFeature].vx_dis[idx_total]);
									fs.vy_dis.push_back(vFeature[iFeature].vy_dis[idx_total]);
									//fs.vvDesc.push_back(vFeature[iFeature].vvDesc[idx_total]);
									vector<double> meanDesc;
									if ((fs.vvDesc.size() == 0) || (vFeature[iFeature].vvDesc.size() == 0))
									{
										if (fs.vvDesc.size() == 0)
										{
											meanDesc = vFeature[iFeature].vvDesc[0];
										}
										else
										{
											meanDesc = fs.vvDesc[0];
										}
									}
									else
									{
										ComputeMeanDescriptor(fs.vvDesc[0], vFeature[iFeature].vvDesc[0], meanDesc);
									}							
									fs.vvDesc.clear();
									fs.vvDesc.push_back(meanDesc);
								}
								else if ((it_total1 == vFeature[iFeature].vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
								{
									int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
									fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
									fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
									fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
									fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
									fs.vx_dis.push_back(vStitchedFeature[iStitchedFeature].vx_dis[idx_total]);
									fs.vy_dis.push_back(vStitchedFeature[iStitchedFeature].vy_dis[idx_total]);
									//fs.vvDesc.push_back(vStitchedFeature[iStitchedFeature].vvDesc[idx_total]);
									vector<double> meanDesc;
									if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
									{
										if (fs.vvDesc.size() == 0)
										{
											meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
										}
										else
										{
											meanDesc = fs.vvDesc[0];
										}
									}
									else
									{
										ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
									}							
									fs.vvDesc.clear();
									fs.vvDesc.push_back(meanDesc);
								}
								temp = iTotalFrame;
							}

							//if (fs.vFrame.size() == 0)
							//{
							//	cout << endl << temp << endl;
							//	for (int i =0; i < vStitchedFeature[iStitchedFeature].vFrame.size(); i++)
							//		cout << vStitchedFeature[iStitchedFeature].vFrame[i] << " ";
							//}
							vStitchedFeature[iStitchedFeature] = fs;
							isInSet = true;
						}
					}
					if (isInSet)
						break;
				}
			}
		}
		if (!isInSet)
			vStitchedFeature.push_back(vFeature[iFeature]);
	}
}

void Stitching(vector<Feature> &vStitchedFeature, Feature feature, int &feature_idx, Feature &newFeature,
				int max_nCameras, int max_nFrames)
{
	bool isInSet = false;
	for (int iStitchedFeature = 0; iStitchedFeature < vStitchedFeature.size(); iStitchedFeature++)
	{
		if (isInSet)
			break;
		//if (vStitchedFeature[iStitchedFeature].vFrame[0] > feature.vFrame[feature.vFrame.size()-1])
		//	continue;
		//if (vStitchedFeature[iStitchedFeature].vFrame[vStitchedFeature[iStitchedFeature].vFrame.size()-1] < feature.vFrame[0])
		//	continue;

		// Find the same frame
		for (int iFeatureFrame = 0; iFeatureFrame < feature.vFrame.size(); iFeatureFrame++)
		{
			//if ((feature.vFrame[iFeatureFrame] < vStitchedFeature[iStitchedFeature].vFrame[0])||
			//	(feature.vFrame[iFeatureFrame] > vStitchedFeature[iStitchedFeature].vFrame[vStitchedFeature[iStitchedFeature].vFrame.size()-1]))
			//	continue;

			vector<int>::const_iterator it = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), 
				feature.vFrame[iFeatureFrame]);
			if (it != vStitchedFeature[iStitchedFeature].vFrame.end())
			{
				int idx = (int) (it - vStitchedFeature[iStitchedFeature].vFrame.begin());
				Point pFeature, pStitchedFeature;
				pFeature.x = feature.vx[iFeatureFrame];
				pFeature.y = feature.vy[iFeatureFrame];
				pStitchedFeature.x = vStitchedFeature[iStitchedFeature].vx[idx];
				pStitchedFeature.y = vStitchedFeature[iStitchedFeature].vy[idx];
				//if (IsSamePoint(pFeature, pStitchedFeature, vFeature[iFeature].vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0]))
				if (IsSamePoint(pFeature, pStitchedFeature))
				{
					//if (IsSamePoint(vFeature[iFeature].vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0]))
					{
						Feature fs;
						fs.id = vStitchedFeature[iStitchedFeature].id;
						fs.r = vStitchedFeature[iStitchedFeature].r;
						fs.g = vStitchedFeature[iStitchedFeature].g;
						fs.b = vStitchedFeature[iStitchedFeature].b;
						int temp = 0;
						for (int iTotalFrame = 0; iTotalFrame < max_nCameras*max_nFrames; iTotalFrame++)
						{
							vector<int>::const_iterator it_total1 = find(feature.vFrame.begin(),feature.vFrame.end(), iTotalFrame);
							vector<int>::const_iterator it_total2 = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), iTotalFrame);
							if ((it_total1 != feature.vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
								fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
								fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
								fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
								fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
								fs.vx_dis.push_back(vStitchedFeature[iStitchedFeature].vx_dis[idx_total]);
								fs.vy_dis.push_back(vStitchedFeature[iStitchedFeature].vy_dis[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							else if ((it_total1 != feature.vFrame.end()) && (it_total2 == vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total1 - feature.vFrame.begin());
								fs.vFrame.push_back(feature.vFrame[idx_total]);
								fs.vCamera.push_back(feature.vCamera[idx_total]);
								fs.vx.push_back(feature.vx[idx_total]);
								fs.vy.push_back(feature.vy[idx_total]);
								fs.vx_dis.push_back(feature.vx_dis[idx_total]);
								fs.vy_dis.push_back(feature.vy_dis[idx_total]);
								//fs.vvDesc.push_back(vFeature[iFeature].vvDesc[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (feature.vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = feature.vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], feature.vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							else if ((it_total1 == feature.vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
								fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
								fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
								fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
								fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
								fs.vx_dis.push_back(vStitchedFeature[iStitchedFeature].vx_dis[idx_total]);
								fs.vy_dis.push_back(vStitchedFeature[iStitchedFeature].vy_dis[idx_total]);
								//fs.vvDesc.push_back(vStitchedFeature[iStitchedFeature].vvDesc[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							temp = iTotalFrame;
						}

						//if (fs.vFrame.size() == 0)
						//{
						//	cout << endl << temp << endl;
						//	for (int i =0; i < vStitchedFeature[iStitchedFeature].vFrame.size(); i++)
						//		cout << vStitchedFeature[iStitchedFeature].vFrame[i] << " ";
						//}
						//vStitchedFeature[iStitchedFeature] = fs;
						newFeature = fs;
						feature_idx = iStitchedFeature;
						isInSet = true;
					}
				}
				if (isInSet)
					break;
			}
		}
	}
	if (!isInSet)
	{
		feature_idx = -1;
		newFeature = feature;
	}
}


void SIFT_Stitching(string path, int nThread)
{
	//string filename;
	string savefile;
	vector<Feature> vFeature, vTempFeature;
	int nFeatures, nTempFeatures;

	vector<int> vnFrames;
	string filename_add = "add_stitchedmeasurement_static.txt";
	//filename = "static_measurement.txt";
	savefile = path + "stitchedmeasurement_static.txt";
	string savefile_desc = path + "descriptors.txt";

	int max_nCameras = 1;
	int nCameras = 1;

	for (int i = 0; i < nThread; i++)
	{
		string savefile_static1 = path + "static_measurement%02d.txt";
		char temp[1000];
		sprintf(temp, savefile_static1.c_str(), i);
		string filename = temp;

		LoadMeasurementData_RGB_DESC_Seq(filename, vFeature);
	}
	int max_nFrames = 0;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		for (int iFrame = 0; iFrame < vFeature[iFeature].vFrame.size(); iFrame++)
		{
			if (max_nFrames < vFeature[iFeature].vFrame[iFrame])
				max_nFrames = vFeature[iFeature].vFrame[iFrame];
		}
	}
	max_nFrames++;
	nFeatures = vFeature.size();
	vnFrames.push_back(max_nFrames);

	vector<Feature> vStitchedFeature;
	vector<Point> vPoint_feature;
	vector<Point> vPoint_stitchedFeature;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (iFeature % 1000 == 1)
			cout << iFeature << " / " << vFeature.size() << endl;
		bool isInSet = false;
		for (int iStitchedFeature = 0; iStitchedFeature < vStitchedFeature.size(); iStitchedFeature++)
		{
			if (isInSet)
				break;
			// Find the same frame
			for (int iFeatureFrame = 0; iFeatureFrame < vFeature[iFeature].vFrame.size(); iFeatureFrame++)
			{
				vector<int>::const_iterator it = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), 
					vFeature[iFeature].vFrame[iFeatureFrame]);
				if (it != vStitchedFeature[iStitchedFeature].vFrame.end())
				{
					int idx = (int) (it - vStitchedFeature[iStitchedFeature].vFrame.begin());
					Point pFeature, pStitchedFeature;
					pFeature.x = vFeature[iFeature].vx[iFeatureFrame];
					pFeature.y = vFeature[iFeature].vy[iFeatureFrame];
					pStitchedFeature.x = vStitchedFeature[iStitchedFeature].vx[idx];
					pStitchedFeature.y = vStitchedFeature[iStitchedFeature].vy[idx];
					if (IsSamePoint(pFeature, pStitchedFeature))
					{
						Feature fs;
						fs.id = vStitchedFeature[iStitchedFeature].id;
						fs.r = vStitchedFeature[iStitchedFeature].r;
						fs.g = vStitchedFeature[iStitchedFeature].g;
						fs.b = vStitchedFeature[iStitchedFeature].b;
						int temp = 0;
						for (int iTotalFrame = 0; iTotalFrame < max_nCameras*max_nFrames; iTotalFrame++)
						{
							vector<int>::const_iterator it_total1 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(), iTotalFrame);
							vector<int>::const_iterator it_total2 = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), iTotalFrame);
							if ((it_total1 != vFeature[iFeature].vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
								fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
								fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
								fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
								fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
								fs.vx_dis.push_back(vStitchedFeature[iStitchedFeature].vx_dis[idx_total]);
								fs.vy_dis.push_back(vStitchedFeature[iStitchedFeature].vy_dis[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							else if ((it_total1 != vFeature[iFeature].vFrame.end()) && (it_total2 == vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total1 - vFeature[iFeature].vFrame.begin());
								fs.vFrame.push_back(vFeature[iFeature].vFrame[idx_total]);
								fs.vCamera.push_back(vFeature[iFeature].vCamera[idx_total]);
								fs.vx.push_back(vFeature[iFeature].vx[idx_total]);
								fs.vy.push_back(vFeature[iFeature].vy[idx_total]);
								fs.vx_dis.push_back(vFeature[iFeature].vx_dis[idx_total]);
								fs.vy_dis.push_back(vFeature[iFeature].vy_dis[idx_total]);
								//fs.vvDesc.push_back(vFeature[iFeature].vvDesc[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (vFeature[iFeature].vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = vFeature[iFeature].vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], vFeature[iFeature].vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							else if ((it_total1 == vFeature[iFeature].vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
								fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
								fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
								fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
								fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
								fs.vx_dis.push_back(vStitchedFeature[iStitchedFeature].vx_dis[idx_total]);
								fs.vy_dis.push_back(vStitchedFeature[iStitchedFeature].vy_dis[idx_total]);
								//fs.vvDesc.push_back(vStitchedFeature[iStitchedFeature].vvDesc[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							temp = iTotalFrame;
						}

						//if (fs.vFrame.size() == 0)
						//{
						//	cout << endl << temp << endl;
						//	for (int i =0; i < vStitchedFeature[iStitchedFeature].vFrame.size(); i++)
						//		cout << vStitchedFeature[iStitchedFeature].vFrame[i] << " ";
						//}
						vStitchedFeature[iStitchedFeature] = fs;
						isInSet = true;
					}
					if (isInSet)
						break;
				}
			}
		}
		if (!isInSet)
			vStitchedFeature.push_back(vFeature[iFeature]);
	}

	SaveMeasurementData_RGB(savefile, vStitchedFeature, max_nFrames, FILESAVE_WRITE_MODE);
	SaveMeasurementData_DESC(savefile_desc, vStitchedFeature, max_nFrames, FILESAVE_WRITE_MODE);
	//SaveMeasurementData(savefile, vStitchedFeature, FILESAVE_WRITE_MODE);
	ResaveMeasurementData(savefile, vnFrames, nCameras);
	return;
}
void SIFT_Stitching(string path)
{
	string filename;
	string savefile;
	vector<Feature> vFeature, vTempFeature;
	int nFeatures, nTempFeatures;
	int nCameras;
	int max_nFrames, max_nCameras;
	vector<int> vnFrames;
	string filename_add = "add_stitchedmeasurement_static.txt";
	filename = "static_measurement.txt";
	savefile = path + "stitchedmeasurement_static.txt";
	string savefile_desc = path + "descriptors.txt";
	LoadMeasurementData_RGB_DESC(path+filename, vFeature, nFeatures, vnFrames, nCameras, max_nFrames, max_nCameras);

	vector<Feature> vStitchedFeature;
	vector<Point> vPoint_feature;
	vector<Point> vPoint_stitchedFeature;
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		if (iFeature % 1000 == 1)
			cout << iFeature << " / " << vFeature.size() << endl;
		bool isInSet = false;
		for (int iStitchedFeature = 0; iStitchedFeature < vStitchedFeature.size(); iStitchedFeature++)
		{

			if (isInSet)
				break;
			// Find the same frame
			for (int iFeatureFrame = 0; iFeatureFrame < vFeature[iFeature].vFrame.size(); iFeatureFrame++)
			{
				vector<int>::const_iterator it = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), 
					vFeature[iFeature].vFrame[iFeatureFrame]);
				if (it != vStitchedFeature[iStitchedFeature].vFrame.end())
				{
					int idx = (int) (it - vStitchedFeature[iStitchedFeature].vFrame.begin());
					Point pFeature, pStitchedFeature;
					pFeature.x = vFeature[iFeature].vx[iFeatureFrame];
					pFeature.y = vFeature[iFeature].vy[iFeatureFrame];
					pStitchedFeature.x = vStitchedFeature[iStitchedFeature].vx[idx];
					pStitchedFeature.y = vStitchedFeature[iStitchedFeature].vy[idx];
					if (IsSamePoint(pFeature, pStitchedFeature))
					{
						Feature fs;
						fs.id = vStitchedFeature[iStitchedFeature].id;
						fs.r = vStitchedFeature[iStitchedFeature].r;
						fs.g = vStitchedFeature[iStitchedFeature].g;
						fs.b = vStitchedFeature[iStitchedFeature].b;
						int temp = 0;
						for (int iTotalFrame = 0; iTotalFrame < max_nCameras*max_nFrames; iTotalFrame++)
						{
							vector<int>::const_iterator it_total1 = find(vFeature[iFeature].vFrame.begin(),vFeature[iFeature].vFrame.end(), iTotalFrame);
							vector<int>::const_iterator it_total2 = find(vStitchedFeature[iStitchedFeature].vFrame.begin(),vStitchedFeature[iStitchedFeature].vFrame.end(), iTotalFrame);
							if ((it_total1 != vFeature[iFeature].vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
								fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
								fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
								fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
								fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							else if ((it_total1 != vFeature[iFeature].vFrame.end()) && (it_total2 == vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total1 - vFeature[iFeature].vFrame.begin());
								fs.vFrame.push_back(vFeature[iFeature].vFrame[idx_total]);
								fs.vCamera.push_back(vFeature[iFeature].vCamera[idx_total]);
								fs.vx.push_back(vFeature[iFeature].vx[idx_total]);
								fs.vy.push_back(vFeature[iFeature].vy[idx_total]);
								//fs.vvDesc.push_back(vFeature[iFeature].vvDesc[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (vFeature[iFeature].vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = vFeature[iFeature].vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], vFeature[iFeature].vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							else if ((it_total1 == vFeature[iFeature].vFrame.end()) && (it_total2 != vStitchedFeature[iStitchedFeature].vFrame.end()))
							{
								int idx_total = (int) (it_total2 - vStitchedFeature[iStitchedFeature].vFrame.begin());
								fs.vFrame.push_back(vStitchedFeature[iStitchedFeature].vFrame[idx_total]);
								fs.vCamera.push_back(vStitchedFeature[iStitchedFeature].vCamera[idx_total]);
								fs.vx.push_back(vStitchedFeature[iStitchedFeature].vx[idx_total]);
								fs.vy.push_back(vStitchedFeature[iStitchedFeature].vy[idx_total]);
								//fs.vvDesc.push_back(vStitchedFeature[iStitchedFeature].vvDesc[idx_total]);
								vector<double> meanDesc;
								if ((fs.vvDesc.size() == 0) || (vStitchedFeature[iStitchedFeature].vvDesc.size() == 0))
								{
									if (fs.vvDesc.size() == 0)
									{
										meanDesc = vStitchedFeature[iStitchedFeature].vvDesc[0];
									}
									else
									{
										meanDesc = fs.vvDesc[0];
									}
								}
								else
								{
									ComputeMeanDescriptor(fs.vvDesc[0], vStitchedFeature[iStitchedFeature].vvDesc[0], meanDesc);
								}							
								fs.vvDesc.clear();
								fs.vvDesc.push_back(meanDesc);
							}
							temp = iTotalFrame;
						}

						//if (fs.vFrame.size() == 0)
						//{
						//	cout << endl << temp << endl;
						//	for (int i =0; i < vStitchedFeature[iStitchedFeature].vFrame.size(); i++)
						//		cout << vStitchedFeature[iStitchedFeature].vFrame[i] << " ";
						//}
						vStitchedFeature[iStitchedFeature] = fs;
						isInSet = true;
					}
					if (isInSet)
						break;
				}
			}
		}
		if (!isInSet)
			vStitchedFeature.push_back(vFeature[iFeature]);
	}

	SaveMeasurementData_RGB(savefile, vStitchedFeature, max_nFrames, FILESAVE_WRITE_MODE);
	SaveMeasurementData_DESC(savefile_desc, vStitchedFeature, max_nFrames, FILESAVE_WRITE_MODE);
	//SaveMeasurementData(savefile, vStitchedFeature, FILESAVE_WRITE_MODE);
	ResaveMeasurementData(savefile, vnFrames, nCameras);
	return;
}

void Iterate_SIFT_STATIC_MP(vector<FrameCamera> &vFC, int currentFC, CvMat *K, CvMat *invK, double omega, vector<Feature> &feature_static, bool display)
{ 
	FrameCamera cFC = vFC[currentFC];

	IplImage *iplImg1 = cvLoadImage(cFC.imageFileName.c_str());
	flann::Matrix<float> descM1(new float[cFC.vSift_desc.size()*128], cFC.vSift_desc.size(), 128);

	for (int iDesc = 0; iDesc < cFC.vSift_desc.size(); iDesc++)
	{
		for (int iDim = 0; iDim < 128; iDim++)
		{
			descM1[iDesc][iDim] = (float) cFC.vSift_desc[iDesc].vDesc[iDim];
		}
	}
	for (int iFeature = 0; iFeature < cFC.vSift_desc.size(); iFeature++)
	{
		Feature fs;
		fs.vCamera.push_back(cFC.cameraID);
		fs.vFrame.push_back(cFC.frameIdx);
		fs.vx.push_back(cFC.vSift_desc[iFeature].x);
		fs.vy.push_back(cFC.vSift_desc[iFeature].y);
		fs.vx_dis.push_back(cFC.vSift_desc[iFeature].dis_x);
		fs.vy_dis.push_back(cFC.vSift_desc[iFeature].dis_y);
		fs.vvDesc.push_back(cFC.vSift_desc[iFeature].vDesc);
		CvScalar s;
		s=cvGet2D(iplImg1,((int)cFC.vSift_desc[iFeature].dis_y),((int)cFC.vSift_desc[iFeature].dis_x));
		fs.b = s.val[0];
		fs.g = s.val[1];
		fs.r = s.val[2];
		feature_static.push_back(fs);
	}
	cvReleaseImage(&iplImg1);
	
	flann::Index<flann::L2<float> > index1(descM1, flann::KDTreeIndexParams(4));
	index1.buildIndex();

	vector<Point> featureSequence;
	for (int iSecondFrame = currentFC+1; iSecondFrame < vFC.size(); iSecondFrame++)
	{
		vector<int> vIdx1, vIdx2;
		int nn = 2;
		int nPoint1 = cFC.vSift_desc.size();
		int nPoint2 = vFC[iSecondFrame].vSift_desc.size();
		
		flann::Matrix<int> result12(new int[nPoint1*nn], nPoint1, nn);
		flann::Matrix<float> dist12(new float[nPoint1*nn], nPoint1, nn);

		flann::Matrix<int> result21(new int[nPoint2*nn], nPoint2, nn);
		flann::Matrix<float> dist21(new float[nPoint2*nn], nPoint2, nn);

		flann::Matrix<float> descM2(new float[vFC[iSecondFrame].vSift_desc.size()*128], vFC[iSecondFrame].vSift_desc.size(), 128);
		for (int iDesc = 0; iDesc < vFC[iSecondFrame].vSift_desc.size(); iDesc++)
		{
			for (int iDim = 0; iDim < 128; iDim++)
			{
				descM2[iDesc][iDim] = (float) vFC[iSecondFrame].vSift_desc[iDesc].vDesc[iDim];
			}
		}

		flann::Index<flann::L2<float> > index2(descM2, flann::KDTreeIndexParams(4));
		index2.buildIndex();

		index2.knnSearch(descM1, result12, dist12, nn, flann::SearchParams(128));
		index1.knnSearch(descM2, result21, dist21, nn, flann::SearchParams(128));
		delete[] descM2.ptr();
		
		for (int iFeature = 0; iFeature < nPoint1; iFeature++)
		{
			float dist1 = dist12[iFeature][0];
			float dist2 = dist12[iFeature][1];

			if (dist1/dist2 < 0.7)
			{
				int idx12 = result12[iFeature][0];

				dist1 = dist21[idx12][0];
				dist2 = dist21[idx12][1];

				if (dist1/dist2 < 0.7)
				{
					int idx21 = result21[idx12][0];
					if (iFeature == idx21)
					{
						vIdx1.push_back(idx21);
						vIdx2.push_back(idx12);
					}
				}

				int idx21 = result21[idx12][0];
				if (iFeature == idx21)
				{
					vIdx1.push_back(idx21);
					vIdx2.push_back(idx12);
				}

				vIdx1.push_back(iFeature);
				vIdx2.push_back(idx12);
			}
		}
		delete[] result12.ptr();
		delete[] result21.ptr();
		delete[] dist12.ptr();
		delete[] dist21.ptr();

		vector<Point> x1, x2;
		for (int iIdx = 0; iIdx < vIdx1.size(); iIdx++)
		{
			Point p1, p2;
			p1.x = cFC.vSift_desc[vIdx1[iIdx]].x;
			p1.y = cFC.vSift_desc[vIdx1[iIdx]].y;

			p2.x = vFC[iSecondFrame].vSift_desc[vIdx2[iIdx]].x;
			p2.y = vFC[iSecondFrame].vSift_desc[vIdx2[iIdx]].y;

			x1.push_back(p1);
			x2.push_back(p2);
		}

		if (x1.size() < 20)
		{
			continue;
		}
		vector<bool> vIsInlier;
		if (GetStaticCorrespondences(x1, x2, vIsInlier) < 20)
		{
			continue;
		}

		vector<int> vTempIdx1, vTempIdx2;
		for (int iIsInlier = 0; iIsInlier < vIsInlier.size(); iIsInlier++)
		{
			if (vIsInlier[iIsInlier])
			{
				if (vTempIdx1.size() > 0)
				{
					if ((vTempIdx1[vTempIdx1.size()-1] == vIdx1[iIsInlier]) && (vTempIdx2[vTempIdx2.size()-1] == vIdx2[iIsInlier]))
					{
						continue;
					}
				}
				vTempIdx1.push_back(vIdx1[iIsInlier]);
				vTempIdx2.push_back(vIdx2[iIsInlier]);
			}
		}
		vIdx1 = vTempIdx1;
		vIdx2 = vTempIdx2;

		for (int iInlier = 0; iInlier < vIdx2.size(); iInlier++)
		{
			feature_static[vIdx1[iInlier]].vCamera.push_back(vFC[iSecondFrame].cameraID);
			feature_static[vIdx1[iInlier]].vFrame.push_back(vFC[iSecondFrame].frameIdx);
			feature_static[vIdx1[iInlier]].vx.push_back(vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].x);
			feature_static[vIdx1[iInlier]].vx_dis.push_back(vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_x);
			feature_static[vIdx1[iInlier]].vy.push_back(vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].y);
			feature_static[vIdx1[iInlier]].vy_dis.push_back(vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_y);
			feature_static[vIdx1[iInlier]].vvDesc.push_back(vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].vDesc);
			//cout << feature_static[vIdx1[iInlier]].r << " " << feature_static[vIdx1[iInlier]].g << " " << feature_static[vIdx1[iInlier]].b << endl;
		}

		if (display)
		{
			IplImage *iplImg2 = cvLoadImage(vFC[iSecondFrame].imageFileName.c_str());

			for (int iInlier = 0; iInlier < vIdx2.size(); iInlier++)
			{
				cvCircle(iplImg2, cvPoint((int)vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_x,(int)vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_y), 8, cvScalar(255,0,0), 1);
				cvLine(iplImg2, cvPoint((int)cFC.vSift_desc[vIdx1[iInlier]].dis_x,(int)cFC.vSift_desc[vIdx1[iInlier]].dis_y), cvPoint((int)vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_x,(int)vFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_y), cvScalar(0,0,0), 3);
			}
			if (iplImg2->width < 1024)
			{
				cvShowImage("SIFT_LOWES", iplImg2);
				cvWaitKey(50);
			}
			else
			{
				double scale = (double)(iplImg2->width)/1024;
				CvSize size = cvSize((int)(iplImg2->width/scale),(int)(iplImg2->height/scale)); 
				IplImage* tmpsize=cvCreateImage(size,IPL_DEPTH_8U,3);
				cvResize(iplImg2,tmpsize,CV_INTER_LINEAR); 
				cvShowImage("SIFT_LOWES", tmpsize);
				cvWaitKey(50);
				cvReleaseImage(&tmpsize);
			}
			cvReleaseImage(&iplImg2);
		}
			
		cout << "Frame 1 : " << cFC.cameraID << " " << cFC.frameIdx << "  Frame 2 : " << vFC[iSecondFrame].cameraID << " " << vFC[iSecondFrame].frameIdx << " /" << vIdx1.size() << endl;
	}

	vector<Feature> vTempFeature;
	for (int iFeature = 0; iFeature < feature_static.size(); iFeature++)
	{
		if (feature_static[iFeature].vCamera.size() > 1)
		{
			vTempFeature.push_back(feature_static[iFeature]);
		}
	}
	feature_static.clear();
	feature_static = vTempFeature;
	vTempFeature.clear();

	delete[] descM1.ptr();
}


void Iterate_SIFT_STATIC(FrameCamera &cFC, vector<FrameCamera> &vCurrentFC, CvMat *K, CvMat *invK, double omega, vector<Feature> &feature_static, bool display)
{ 
	IplImage *iplImg1 = cvLoadImage(cFC.imageFileName.c_str());
	flann::Matrix<float> descM1(new float[cFC.vSift_desc.size()*128], cFC.vSift_desc.size(), 128);

	for (int iDesc = 0; iDesc < cFC.vSift_desc.size(); iDesc++)
	{
		for (int iDim = 0; iDim < 128; iDim++)
		{
			descM1[iDesc][iDim] = (float) cFC.vSift_desc[iDesc].vDesc[iDim];
		}
	}
	for (int iFeature = 0; iFeature < cFC.vSift_desc.size(); iFeature++)
	{
		Feature fs;
		fs.vCamera.push_back(cFC.cameraID);
		fs.vFrame.push_back(cFC.frameIdx);
		fs.vx.push_back(cFC.vSift_desc[iFeature].x);
		fs.vy.push_back(cFC.vSift_desc[iFeature].y);
		fs.vx_dis.push_back(cFC.vSift_desc[iFeature].dis_x);
		fs.vy_dis.push_back(cFC.vSift_desc[iFeature].dis_y);
		fs.vvDesc.push_back(cFC.vSift_desc[iFeature].vDesc);
		CvScalar s;
		s=cvGet2D(iplImg1,((int)cFC.vSift_desc[iFeature].dis_y),((int)cFC.vSift_desc[iFeature].dis_x));
		fs.b = s.val[0];
		fs.g = s.val[1];
		fs.r = s.val[2];
		feature_static.push_back(fs);
	}
	cvReleaseImage(&iplImg1);
	
	flann::Index<flann::L2<float> > index1(descM1, flann::KDTreeIndexParams(4));
	index1.buildIndex();

	vector<Point> featureSequence;
	for (int iSecondFrame = 0; iSecondFrame < vCurrentFC.size(); iSecondFrame++)
	{
		vector<int> vIdx1, vIdx2;
		int nn = 2;
		int nPoint1 = cFC.vSift_desc.size();
		int nPoint2 = vCurrentFC[iSecondFrame].vSift_desc.size();
		
		flann::Matrix<int> result12(new int[nPoint1*nn], nPoint1, nn);
		flann::Matrix<float> dist12(new float[nPoint1*nn], nPoint1, nn);

		flann::Matrix<int> result21(new int[nPoint2*nn], nPoint2, nn);
		flann::Matrix<float> dist21(new float[nPoint2*nn], nPoint2, nn);

		flann::Matrix<float> descM2(new float[vCurrentFC[iSecondFrame].vSift_desc.size()*128], vCurrentFC[iSecondFrame].vSift_desc.size(), 128);
		for (int iDesc = 0; iDesc < vCurrentFC[iSecondFrame].vSift_desc.size(); iDesc++)
		{
			for (int iDim = 0; iDim < 128; iDim++)
			{
				descM2[iDesc][iDim] = (float) vCurrentFC[iSecondFrame].vSift_desc[iDesc].vDesc[iDim];
			}
		}

		flann::Index<flann::L2<float> > index2(descM2, flann::KDTreeIndexParams(4));
		index2.buildIndex();

		index2.knnSearch(descM1, result12, dist12, nn, flann::SearchParams(128));
		index1.knnSearch(descM2, result21, dist21, nn, flann::SearchParams(128));
		delete[] descM2.ptr();
		
		for (int iFeature = 0; iFeature < nPoint1; iFeature++)
		{
			float dist1 = dist12[iFeature][0];
			float dist2 = dist12[iFeature][1];

			if (dist1/dist2 < 0.7)
			{
				int idx12 = result12[iFeature][0];

				dist1 = dist21[idx12][0];
				dist2 = dist21[idx12][1];

				if (dist1/dist2 < 0.7)
				{
					int idx21 = result21[idx12][0];
					if (iFeature == idx21)
					{
						vIdx1.push_back(idx21);
						vIdx2.push_back(idx12);
					}
				}

				int idx21 = result21[idx12][0];
				if (iFeature == idx21)
				{
					vIdx1.push_back(idx21);
					vIdx2.push_back(idx12);
				}

				vIdx1.push_back(iFeature);
				vIdx2.push_back(idx12);
			}
		}
		delete[] result12.ptr();
		delete[] result21.ptr();
		delete[] dist12.ptr();
		delete[] dist21.ptr();

		vector<Point> x1, x2;
		for (int iIdx = 0; iIdx < vIdx1.size(); iIdx++)
		{
			Point p1, p2;
			p1.x = cFC.vSift_desc[vIdx1[iIdx]].x;
			p1.y = cFC.vSift_desc[vIdx1[iIdx]].y;

			p2.x = vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iIdx]].x;
			p2.y = vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iIdx]].y;

			x1.push_back(p1);
			x2.push_back(p2);
		}

		if (x1.size() < 20)
		{
			continue;
		}
		vector<bool> vIsInlier;
		if (GetStaticCorrespondences(x1, x2, vIsInlier) < 20)
		{
			continue;
		}

		vector<int> vTempIdx1, vTempIdx2;
		for (int iIsInlier = 0; iIsInlier < vIsInlier.size(); iIsInlier++)
		{
			if (vIsInlier[iIsInlier])
			{
				if (vTempIdx1.size() > 0)
				{
					if ((vTempIdx1[vTempIdx1.size()-1] == vIdx1[iIsInlier]) && (vTempIdx2[vTempIdx2.size()-1] == vIdx2[iIsInlier]))
					{
						continue;
					}
				}
				vTempIdx1.push_back(vIdx1[iIsInlier]);
				vTempIdx2.push_back(vIdx2[iIsInlier]);
			}
		}
		vIdx1 = vTempIdx1;
		vIdx2 = vTempIdx2;

		for (int iInlier = 0; iInlier < vIdx2.size(); iInlier++)
		{
			feature_static[vIdx1[iInlier]].vCamera.push_back(vCurrentFC[iSecondFrame].cameraID);
			feature_static[vIdx1[iInlier]].vFrame.push_back(vCurrentFC[iSecondFrame].frameIdx);
			feature_static[vIdx1[iInlier]].vx.push_back(vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].x);
			feature_static[vIdx1[iInlier]].vx_dis.push_back(vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_x);
			feature_static[vIdx1[iInlier]].vy.push_back(vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].y);
			feature_static[vIdx1[iInlier]].vy_dis.push_back(vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_y);
			feature_static[vIdx1[iInlier]].vvDesc.push_back(vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].vDesc);
			//cout << feature_static[vIdx1[iInlier]].r << " " << feature_static[vIdx1[iInlier]].g << " " << feature_static[vIdx1[iInlier]].b << endl;
		}

		if (display)
		{
			IplImage *iplImg2 = cvLoadImage(vCurrentFC[iSecondFrame].imageFileName.c_str());

			for (int iInlier = 0; iInlier < vIdx2.size(); iInlier++)
			{
				cvCircle(iplImg2, cvPoint((int)vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_x,(int)vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_y), 8, cvScalar(255,0,0), 1);
				cvLine(iplImg2, cvPoint((int)cFC.vSift_desc[vIdx1[iInlier]].dis_x,(int)cFC.vSift_desc[vIdx1[iInlier]].dis_y), cvPoint((int)vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_x,(int)vCurrentFC[iSecondFrame].vSift_desc[vIdx2[iInlier]].dis_y), cvScalar(0,0,0), 3);
			}
			if (iplImg2->width < 1024)
			{
				cvShowImage("SIFT_LOWES", iplImg2);
				cvWaitKey(50);
			}
			else
			{
				double scale = (double)(iplImg2->width)/1024;
				CvSize size = cvSize((int)(iplImg2->width/scale),(int)(iplImg2->height/scale)); 
				IplImage* tmpsize=cvCreateImage(size,IPL_DEPTH_8U,3);
				cvResize(iplImg2,tmpsize,CV_INTER_LINEAR); 
				cvShowImage("SIFT_LOWES", tmpsize);
				cvWaitKey(50);
				cvReleaseImage(&tmpsize);
			}
			cvReleaseImage(&iplImg2);
		}
			
		cout << "Frame 1 : " << cFC.cameraID << " " << cFC.frameIdx << "  Frame 2 : " << vCurrentFC[iSecondFrame].cameraID << " " << vCurrentFC[iSecondFrame].frameIdx << " /" << vIdx1.size() << endl;
	}

	vector<Feature> vTempFeature;
	for (int iFeature = 0; iFeature < feature_static.size(); iFeature++)
	{
		if (feature_static[iFeature].vCamera.size() > 1)
		{
			vTempFeature.push_back(feature_static[iFeature]);
		}
	}
	feature_static.clear();
	feature_static = vTempFeature;
	vTempFeature.clear();

	delete[] descM1.ptr();
}

//void Iterate_SIFT_STATIC(FrameCamera cFC, vector<FrameCamera> vCurrentFC, CvMat *K, CvMat *invK, double omega, vector<Feature> &feature, bool display)
//{
//	IplImage *iplImg1 = cvLoadImage(cFC.imageFileName.c_str());
//	string keyFile = cFC.imageFileName;
//	keyFile[keyFile.size()-3] = 'k';
//	keyFile[keyFile.size()-2] = 'e';
//	keyFile[keyFile.size()-1] = 'y';
//
//	vector<SIFT_Descriptor> vSift_desc1;
//	LoadSIFTData(keyFile, vSift_desc1);
//
//	float *desc1 = (float *) malloc(vSift_desc1.size()*128*sizeof(float));
//	for (int iDesc = 0; iDesc < vSift_desc1.size(); iDesc++)
//	{
//		for (int iDim = 0; iDim < 128; iDim++)
//		{
//			desc1[iDesc*128+iDim] = (float) vSift_desc1[iDesc].vDesc[iDim];
//		}
//	}
//
//	float speedup;
//	struct FLANNParameters p = DEFAULT_FLANN_PARAMETERS;
//	p.algorithm = FLANN_INDEX_AUTOTUNED;
//	p.target_precision = 0.9;
//
//	flann_index_t dataset_index = flann_build_index_float(desc1, vSift_desc1.size(), 128, &speedup, &p);
//	
//	vector<double> vx1, vy1;
//	vector<double> dis_vx1, dis_vy1;
//	for (int isift = 0; isift < vSift_desc1.size(); isift++)
//	{
//		vx1.push_back(vSift_desc1[isift].x);
//		vy1.push_back(vSift_desc1[isift].y);
//	}
//	Undistortion(K, invK, omega, vx1, vy1);
//	for (int isift = 0; isift < vSift_desc1.size(); isift++)
//	{
//		vSift_desc1[isift].x = vx1[isift];
//		vSift_desc1[isift].y = vy1[isift];
//	}
//
//	
//	//CvMat *desc1 = cvCreateMat(vSift_desc1.size(), 128, CV_32FC1);
//	//for (int iFeature = 0; iFeature < vSift_desc1.size(); iFeature++)
//	//{
//	//	for (int iDim = 0; iDim < 128; iDim++)
//	//	{
//	//		cvSetReal2D(desc1, iFeature, iDim, vSift_desc1[iFeature].vDesc[iDim]);
//	//	}
//	//}
//	vector<Point> featureSequence;
//
//	for (int iSecondFrame = 0; iSecondFrame < vCurrentFC.size(); iSecondFrame++)
//	{
//		IplImage *iplImg2 = cvLoadImage(vCurrentFC[iSecondFrame].imageFileName.c_str());
//		keyFile = vCurrentFC[iSecondFrame].imageFileName;
//		keyFile[keyFile.size()-3] = 'k';s
//		keyFile[keyFile.size()-2] = 'e';
//		keyFile[keyFile.size()-1] = 'y';
//
//		vector<SIFT_Descriptor> vSift_desc2;
//		LoadSIFTData(keyFile, vSift_desc2);
//
//		float *desc2 = (float *) malloc(vSift_desc2.size()*128*sizeof(float));
//		for (int iDesc = 0; iDesc < vSift_desc2.size(); iDesc++)
//		{
//			for (int iDim = 0; iDim < 128; iDim++)
//			{
//				desc2[iDesc*128+iDim] = (float) vSift_desc2[iDesc].vDesc[iDim];
//			}
//		}
//	
//
//		vector<double> vx2, vy2;
//		vector<double> dis_vx2, dis_vy2;
//		for (int isift = 0; isift < vSift_desc2.size(); isift++)
//		{
//			vx2.push_back(vSift_desc2[isift].x);
//			vy2.push_back(vSift_desc2[isift].y);
//		}
//		dis_vx2 = vx2;
//		dis_vy2 = vy2;
//		Undistortion(K, invK, omega, vx2, vy2);
//		for (int isift = 0; isift < vSift_desc2.size(); isift++)
//		{
//			vSift_desc2[isift].x = vx2[isift];
//			vSift_desc2[isift].y = vy2[isift];
//		}
//		vector<int> vIdx1, vIdx2;
//
//		int nn = 2;
//		int nPoint1 = vSift_desc1.size();
//		int nPoint2 = vSift_desc2.size();
//		int *result21 = (int*) malloc(nPoint2*nn*sizeof(int));
//		float *dist21 = (float*) malloc(nPoint2*nn*sizeof(float));
//		flann_find_nearest_neighbors_index(dataset_index, desc2, nPoint2, result21, dist21, nn, &p);
//
//		for (int iFeature = 0; iFeature < nPoint2; iFeature++)
//		{
//			double dist1 = dist21[iFeature*nn+0];
//			double dist2 = dist21[iFeature*nn+1];
//
//			if (dist1/dist2 < 0.7)
//			{
//				vIdx1.push_back(result21[iFeature*nn]);
//				vIdx2.push_back(iFeature);
//			}
//		}
//		free(result21);
//		free(dist21);
//		free(desc2);
//
//		vector<Point> x1, x2;
//		for (int iIdx = 0; iIdx < vIdx1.size(); iIdx++)
//		{
//			Point p1, p2;
//			p1.x = vSift_desc1[vIdx1[iIdx]].x;
//			p1.y = vSift_desc1[vIdx1[iIdx]].y;
//
//			p2.x = vSift_desc2[vIdx2[iIdx]].x;
//			p2.y = vSift_desc2[vIdx2[iIdx]].y;
//
//			x1.push_back(p1);
//			x2.push_back(p2);
//		}
//
//		if (x1.size() < 20)
//		{
//			cvReleaseImage(&iplImg2);
//			continue;
//		}
//		vector<bool> vIsInlier;
//		if (GetStaticCorrespondences(x1, x2, vIsInlier) < 20)
//		{
//			cvReleaseImage(&iplImg2);
//			continue;
//		}
//		
//		vector<SIFT_Descriptor> vNewSIFT1, vNewSIFT2;
//		for (int iIdx = 0; iIdx < vIsInlier.size(); iIdx++)
//		{
//			if (vIsInlier[iIdx])
//			{
//				vNewSIFT1.push_back(vSift_desc1[vIdx1[iIdx]]);
//				vNewSIFT2.push_back(vSift_desc2[vIdx2[iIdx]]);				
//			}
//		}
//
//		vector<Point> vTempPoint;
//		for (int iNewSIFT1 = 0; iNewSIFT1 < vNewSIFT1.size(); iNewSIFT1++)
//		{
//			Point p;
//			p.x = vNewSIFT1[iNewSIFT1].x;
//			p.y = vNewSIFT1[iNewSIFT1].y;
//			vTempPoint.push_back(p);
//		}
//
//		// Registration
//		for (vector<Point>::iterator iit = vTempPoint.begin(); iit < vTempPoint.end(); iit++)
//		{
//			vector<Point>::const_iterator it = std::search(featureSequence.begin(),featureSequence.end(),iit, iit+1, IsSamePoint);
//			int iPoint = int(iit-vTempPoint.begin());
//			if (it != featureSequence.end())
//			{
//				int idx = int(it-featureSequence.begin());
//
//				//if (((int)vPoint2[iPoint].y > 220) && ((int)vPoint2[iPoint].x < 60))
//				//	continue;
//				feature[idx].vx.push_back(vNewSIFT2[iPoint].x);
//				feature[idx].vy.push_back(vNewSIFT2[iPoint].y);
//				feature[idx].vvDesc.push_back(vNewSIFT2[iPoint].vDesc);
//				feature[idx].vCamera.push_back(vCurrentFC[iSecondFrame].cameraID);
//				feature[idx].vFrame.push_back(vCurrentFC[iSecondFrame].frameIdx);
//			}
//			else
//			{
//				Feature fs;
//				fs.vCamera.push_back(cFC.cameraID);
//				fs.vCamera.push_back(vCurrentFC[iSecondFrame].cameraID);
//				fs.vFrame.push_back(cFC.frameIdx);
//				fs.vFrame.push_back(vCurrentFC[iSecondFrame].frameIdx);
//				fs.vx.push_back(vNewSIFT1[iPoint].x);
//				fs.vx.push_back(vNewSIFT2[iPoint].x);
//				fs.vy.push_back(vNewSIFT1[iPoint].y);
//				fs.vy.push_back(vNewSIFT2[iPoint].y);
//				fs.vvDesc.push_back(vNewSIFT1[iPoint].vDesc);
//				fs.vvDesc.push_back(vNewSIFT2[iPoint].vDesc);
//				CvScalar s;
//				s=cvGet2D(iplImg1,((int)vNewSIFT1[iPoint].dis_x),((int)vNewSIFT1[iPoint].dis_y));
//				fs.b = s.val[0];
//				fs.g = s.val[1];
//				fs.r = s.val[2];
//
//				feature.push_back(fs);
//				Point p;
//				p.x = vNewSIFT1[iPoint].x;
//				p.y = vNewSIFT1[iPoint].y;
//				featureSequence.push_back(p);
//			}
//		}
//
//		if (display)
//		{
//			//IplImage *iplImg = cvLoadImage(vCurrentFC[iSecondFrame].imageFileName.c_str());
//			//IplImage *iplImg = cvLoadImage("temp2.jpg");
//			//cvCircle(iplImg, cvPoint(56,227), 8, cvScalar(255,255,0), 10);
//			for (int i = 0; i < vNewSIFT1.size(); i++)
//			{
//				//if (((int)vPoint2[i].y > 220) && ((int)vPoint2[i].x < 60))
//				//	continue;
//				//cvCircle(iplImg, cvPoint(vPoint1[i].x,(int)vPoint1[i].y), 8, cvScalar(255,0,0), 1);
//				cvCircle(iplImg2, cvPoint((int)vNewSIFT2[i].dis_y,(int)vNewSIFT2[i].dis_x), 8, cvScalar(255,0,0), 1);
//				cvLine(iplImg2, cvPoint((int)vNewSIFT1[i].dis_y,(int)vNewSIFT1[i].dis_x), cvPoint((int)vNewSIFT2[i].dis_y,(int)vNewSIFT2[i].dis_x), cvScalar(0,0,0), 3);
//			}
//			if (iplImg2->width < 1024)
//			{
//				cvShowImage("SIFT_LOWES", iplImg2);
//
//				cvWaitKey(50);
//				cvReleaseImage(&iplImg2);
//			}
//			else
//			{
//				double scale = (double)(iplImg2->width)/1024;
//				CvSize size = cvSize((int)(iplImg2->width/scale),(int)(iplImg2->height/scale)); 
//				IplImage* tmpsize=cvCreateImage(size,IPL_DEPTH_8U,3);
//				cvResize(iplImg2,tmpsize,CV_INTER_LINEAR); 
//				cvShowImage("SIFT_LOWES", tmpsize);
//				//cout << "Frame 1 : " << cFC.cameraID << " " << cFC.frameIdx << "  Frame 2 : " << vCurrentFC[iSecondFrame].cameraID << " " << vCurrentFC[iSecondFrame].frameIdx << " /" << vPoint1.size() << endl;
//				cvWaitKey(50);
//				cvReleaseImage(&tmpsize);
//				cvReleaseImage(&iplImg2);
//			}	
//		}
//		cvReleaseImage(&iplImg2);
//		cout << "Frame 1 : " << cFC.cameraID << " " << cFC.frameIdx << "  Frame 2 : " << vCurrentFC[iSecondFrame].cameraID << " " << vCurrentFC[iSecondFrame].frameIdx << " /" << vNewSIFT1.size() << endl;
//	}
//	cvReleaseImage(&iplImg1);
//	//cvReleaseMat(&desc1);
//	free(desc1);
//}

void DoSIFT(CvMat *desc1, CvMat *desc2, vector<int> &vPoint1, vector<int> &vPoint2)
{
	//CvFeatureTree *ft1 = cvCreateFeatureTree(desc1);
	//CvMat *match21 = cvCreateMat(desc2->rows, 2, CV_32S);
	//CvMat *dist21 = cvCreateMat(desc2->rows, 2, CV_64FC1);
	////cvFindFeatures(ft1, desc2, match21, dist21, 2, 100);
	//cvFindFeatures(ft1, desc2, match21, dist21, 2);

	//CvFeatureTree *ft2 = cvCreateFeatureTree(desc2);
	//CvMat *match12 = cvCreateMat(desc1->rows, 2, CV_32S);
	//CvMat *dist12 = cvCreateMat(desc1->rows, 2, CV_64FC1);
	////cvFindFeatures(ft2, desc1, match12, dist12, 2, 100);
	//cvFindFeatures(ft2, desc1, match12, dist12, 2);

	//for (int iFeature = 0; iFeature < match12->rows; iFeature++)
	//{
	//	double dist1 = cvGetReal2D(dist12, iFeature, 0);
	//	double dist2 = cvGetReal2D(dist12, iFeature, 1);
	//	int ele1 = 0;
	//	if (dist1 > dist2)
	//	{
	//		ele1 = 1;
	//		dist1 = cvGetReal2D(dist12, iFeature, 1);
	//		dist2 = cvGetReal2D(dist12, iFeature, 0);
	//	}

	//	if (dist1/dist2 < 0.7)
	//	{
	//		int idx12 = cvGetReal2D(match12, iFeature, ele1);

	//		dist1 = cvGetReal2D(dist21, idx12, 0);
	//		dist2 = cvGetReal2D(dist21, idx12, 1);
	//		int ele2 = 0;
	//		if (dist1 > dist2)
	//		{
	//			ele2 = 1;
	//			dist1 = cvGetReal2D(dist21, idx12, 1);
	//			dist2 = cvGetReal2D(dist21, idx12, 0);
	//		}
	//		//if (dist1/dist2 < 0.7)
	//		//{
	//		//	int idx21 = cvGetReal2D(match21, idx12, ele2);
	//		//	if (iFeature == idx21)
	//		//	{
	//		//		vPoint1.push_back(idx21);
	//		//		vPoint2.push_back(idx12);
	//		//	}
	//		//}

	//		int idx21 = cvGetReal2D(match21, idx12, ele2);
	//		if (iFeature == idx21)
	//		{
	//			vPoint1.push_back(idx21);
	//			vPoint2.push_back(idx12);
	//		}
	//	}
	//}

	//cvReleaseFeatureTree(ft1);
	//cvReleaseMat(&match21);
	//cvReleaseMat(&dist21);
	//cvReleaseFeatureTree(ft2);
	//cvReleaseMat(&match12);
	//cvReleaseMat(&dist12);
}

void DoSIFT(float *desc1, float *desc2, int nPoint1, int nPoint2, vector<int> &vPoint1, vector<int> &vPoint2)
{
	//int nn = 2;
	//int *result21 = (int*) malloc(nPoint2*nn*sizeof(int));
	//float *dist21 = (float*) malloc(nPoint2*nn*sizeof(float));

	//int *result12 = (int*) malloc(nPoint1*nn*sizeof(int));
	//float *dist12 = (float*) malloc(nPoint1*nn*sizeof(float));

	//struct FLANNParameters p = DEFAULT_FLANN_PARAMETERS;
	//p.algorithm = FLANN_INDEX_AUTOTUNED;
	//p.target_precision = 0.9;
	//
	//flann_find_nearest_neighbors(desc1, nPoint1, 128, desc2, nPoint2, result21, dist21, nn, &p);
	//flann_find_nearest_neighbors(desc2, nPoint2, 128, desc1, nPoint1, result12, dist12, nn, &p);

	//for (int iFeature = 0; iFeature < nPoint1; iFeature++)
	//{
	//	double dist1 = dist12[iFeature*nn+0];
	//	double dist2 = dist12[iFeature*nn+1];
	//	int ele1 = 0;
	//	if (dist1 > dist2)
	//	{
	//		ele1 = 1;
	//		dist1 = dist12[iFeature*nn+1];
	//		dist2 = dist12[iFeature*nn+0];
	//	}

	//	if (dist1/dist2 < 0.7)
	//	{
	//		int idx12 = result12[iFeature*nn+ele1];

	//		dist1 = dist21[idx12*nn+0];
	//		dist2 = dist21[idx12*nn+1];
	//		int ele2 = 0;
	//		if (dist1 > dist2)
	//		{
	//			ele2 = 1;
	//			dist1 = dist21[idx12*nn+1];
	//			dist2 = dist21[idx12*nn+0];
	//		}
	//		//if (dist1/dist2 < 0.7)
	//		//{
	//		//	int idx21 = cvGetReal2D(match21, idx12, ele2);
	//		//	if (iFeature == idx21)
	//		//	{
	//		//		vPoint1.push_back(idx21);
	//		//		vPoint2.push_back(idx12);
	//		//	}
	//		//}

	//		int idx21 = result21[idx12*nn+ele2];
	//		if (iFeature == idx21)
	//		{
	//			vPoint1.push_back(idx21);
	//			vPoint2.push_back(idx12);
	//		}
	//	}
	//}

	//free(result21);
	//free(result12);
	//free(dist12);
	//free(dist21);



	////CvFeatureTree *ft1 = cvCreateFeatureTree(desc1);
	////CvMat *match21 = cvCreateMat(desc2->rows, 2, CV_32S);
	////CvMat *dist21 = cvCreateMat(desc2->rows, 2, CV_64FC1);
	//////cvFindFeatures(ft1, desc2, match21, dist21, 2, 100);
	////cvFindFeatures(ft1, desc2, match21, dist21, 2);

	////CvFeatureTree *ft2 = cvCreateFeatureTree(desc2);
	////CvMat *match12 = cvCreateMat(desc1->rows, 2, CV_32S);
	////CvMat *dist12 = cvCreateMat(desc1->rows, 2, CV_64FC1);
	//////cvFindFeatures(ft2, desc1, match12, dist12, 2, 100);
	////cvFindFeatures(ft2, desc1, match12, dist12, 2);

	////for (int iFeature = 0; iFeature < match12->rows; iFeature++)
	////{
	////	double dist1 = cvGetReal2D(dist12, iFeature, 0);
	////	double dist2 = cvGetReal2D(dist12, iFeature, 1);
	////	int ele1 = 0;
	////	if (dist1 > dist2)
	////	{
	////		ele1 = 1;
	////		dist1 = cvGetReal2D(dist12, iFeature, 1);
	////		dist2 = cvGetReal2D(dist12, iFeature, 0);
	////	}

	////	if (dist1/dist2 < 0.7)
	////	{
	////		int idx12 = cvGetReal2D(match12, iFeature, ele1);

	////		dist1 = cvGetReal2D(dist21, idx12, 0);
	////		dist2 = cvGetReal2D(dist21, idx12, 1);
	////		int ele2 = 0;
	////		if (dist1 > dist2)
	////		{
	////			ele2 = 1;
	////			dist1 = cvGetReal2D(dist21, idx12, 1);
	////			dist2 = cvGetReal2D(dist21, idx12, 0);
	////		}
	////		//if (dist1/dist2 < 0.7)
	////		//{
	////		//	int idx21 = cvGetReal2D(match21, idx12, ele2);
	////		//	if (iFeature == idx21)
	////		//	{
	////		//		vPoint1.push_back(idx21);
	////		//		vPoint2.push_back(idx12);
	////		//	}
	////		//}

	////		int idx21 = cvGetReal2D(match21, idx12, ele2);
	////		if (iFeature == idx21)
	////		{
	////			vPoint1.push_back(idx21);
	////			vPoint2.push_back(idx12);
	////		}
	////	}
	////}

	////cvReleaseFeatureTree(ft1);
	////cvReleaseMat(&match21);
	////cvReleaseMat(&dist21);
	////cvReleaseFeatureTree(ft2);
	////cvReleaseMat(&match12);
	////cvReleaseMat(&dist12);
}

int GetStaticCorrespondences(vector<Point> x1, vector<Point> x2, vector<bool> &vIsInlier)
{
	vector<int> vInlierID;
	CvMat *cx1 = cvCreateMat(x1.size(), 2, CV_32FC1);
	CvMat *cx2 = cvCreateMat(x1.size(), 2, CV_32FC1);
	for (int ix = 0; ix < x1.size(); ix++)
	{
		cvSetReal2D(cx1, ix, 0, x1[ix].x);
		cvSetReal2D(cx1, ix, 1, x1[ix].y);
		cvSetReal2D(cx2, ix, 0, x2[ix].x);
		cvSetReal2D(cx2, ix, 1, x2[ix].y);
	}

	vector<cv::Point2f> points1(x1.size());
	vector<cv::Point2f> points2(x2.size());
	for (int ip = 0; ip < x1.size(); ip++)
	{
		cv::Point2f p1, p2;
		p1.x = x1[ip].x;
		p1.y = x1[ip].y;
		p2.x = x2[ip].x;
		p2.y = x2[ip].y;
		points1[ip] = p1;
		points2[ip] = p2;
	}

	CvMat *status = cvCreateMat(1,cx1->rows,CV_8UC1);
	CvMat *F = cvCreateMat(3,3,CV_32FC1);
	int n = cvFindFundamentalMat(cx1, cx2, F, CV_FM_LMEDS, 1, 0.99, status);
	//cv::Mat FundamentalMatrix = cv::findFundamentalMat(points1, points2, cv::FM_LMEDS, 3, 0.99);
	if (n != 1)
	{
		cvReleaseMat(&status);
		cvReleaseMat(&F);
		cvReleaseMat(&cx1);
		cvReleaseMat(&cx2);
		return 0;
	}
	int nP=0;
	double ave = 0;
	for (int i = 0; i < cx1->rows; i++)
	{
		if (cvGetReal2D(status, 0, i) == 1)
		{
			//vIsInlier.push_back(true);
			nP++;
			CvMat *xM2 = cvCreateMat(1,3,CV_32FC1);
			CvMat *xM1 = cvCreateMat(3,1,CV_32FC1);
			CvMat *s = cvCreateMat(1,1, CV_32FC1);
			cvSetReal2D(xM2, 0, 0, x2[i].x);
			cvSetReal2D(xM2, 0, 1, x2[i].y);
			cvSetReal2D(xM2, 0, 2, 1);
			cvSetReal2D(xM1, 0, 0, x1[i].x);
			cvSetReal2D(xM1, 1, 0, x1[i].y);
			cvSetReal2D(xM1, 2, 0, 1);
			cvMatMul(xM2, F, xM2);
			cvMatMul(xM2, xM1, s);			

			double l1 = cvGetReal2D(xM2, 0, 0);
			double l2 = cvGetReal2D(xM2, 0, 1);
			double l3 = cvGetReal2D(xM2, 0, 2);

			double dist = abs(cvGetReal2D(s, 0, 0))/sqrt(l1*l1+l2*l2);

			if (dist < 5)
			{
				vIsInlier.push_back(true);
			}
			else
			{
				vIsInlier.push_back(false);
			}
			ave += dist;

			cvReleaseMat(&xM2);
			cvReleaseMat(&xM1);
			cvReleaseMat(&s);
		}
		else
		{
			vIsInlier.push_back(false);
		}
	}
	//cout << ave/nP << endl;

	cvReleaseMat(&status);
	cvReleaseMat(&F);
	cvReleaseMat(&cx1);
	cvReleaseMat(&cx2);
	if (ave/nP > 10)
		return 0;
	return nP;
}