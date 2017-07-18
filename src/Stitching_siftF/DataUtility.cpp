#include "DataUtility.h"
void LoadStaticMeasurementData(string filename, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras)
{
	PrintAlgorithm("Load Static Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nFrames;
	file >> a >> nCameras;
	file >> a >> nFeatures;
	int featureId = 0;
	//while (!file.eof())
	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		file >> nVisibleFrame;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;
			double x, y;

			file >> cam >> frame >> x >> y;
			vCam.push_back(cam);
			vFrame.push_back(frame);
			vx.push_back(x);	vy.push_back(y);
		}
		feature.id = featureId++;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;

		//feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
	}
	//vFeature.pop_back();
	file.close();
}

void LoadSIFTData(string filename, vector<SIFT_Descriptor> &vDescriptor)
{
	PrintAlgorithm("Load SIFT (LOWES) Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	int nPoints, nDim;
	file.open(filename.c_str(), ifstream::in);
	file >> nPoints >> nDim;

	for (int j = 0; j < nPoints; j++)
	{
		SIFT_Descriptor descriptor;
		double x, y, dummy;
		file >> y >> x >> dummy >> dummy;
		vector<double> desc;
		for (int iDim = 0; iDim < nDim; iDim++)
		{
			file >> dummy;
			desc.push_back(dummy);
		}
		descriptor.x = x;
		descriptor.y = y;
		descriptor.dis_x = x;
		descriptor.dis_y = y;
		descriptor.vDesc = desc;
		descriptor.id = j;

		vDescriptor.push_back(descriptor);
	}
	file.close();
}

void LoadSIFTData_int(string filename, vector<SIFT_Descriptor> &vDescriptor)
{
	PrintAlgorithm("Load SIFT (LOWES) Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	int nPoints, nDim;
	file.open(filename.c_str(), ifstream::in);
	file >> nPoints >> nDim;

	for (int j = 0; j < nPoints; j++)
	{
		SIFT_Descriptor descriptor;
		double x, y, dummy;
		file >> y >> x >> dummy >> dummy;
		vector<int> desc;
		for (int iDim = 0; iDim < nDim; iDim++)
		{
			file >> dummy;
			desc.push_back(dummy);
		}
		descriptor.x = x;
		descriptor.y = y;
		descriptor.dis_x = x;
		descriptor.dis_y = y;
		descriptor.vDesc_int = desc;
		descriptor.id = j;

		vDescriptor.push_back(descriptor);
	}
	file.close();
}

void LoadSIFTData_subsampling(string filename, vector<SIFT_Descriptor> &vDescriptor)
{
	PrintAlgorithm("Load SIFT (LOWES) Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	int nPoints, nDim;
	file.open(filename.c_str(), ifstream::in);
	file >> nPoints >> nDim;

	for (int j = 0; j < nPoints; j++)
	{
		SIFT_Descriptor descriptor;
		double x, y, dummy;
		file >> y >> x >> dummy >> dummy;
		vector<int> desc;
		for (int iDim = 0; iDim < nDim; iDim++)
		{
			file >> dummy;
			desc.push_back(dummy);
		}

		if (j%2!=1)
		{
			desc.clear();
			continue;
		}
		descriptor.x = x;
		descriptor.y = y;
		descriptor.dis_x = x;
		descriptor.dis_y = y;
		descriptor.vDesc_int = desc;
		descriptor.id = j;

		vDescriptor.push_back(descriptor);
	}
	cout << vDescriptor.size() << " " << nPoints << endl;
	file.close();
}

void LoadSIFTData_ScaleDirection(string filename, vector<SIFT_Descriptor> &vDescriptor)
{
	PrintAlgorithm("Load SIFT (LOWES) Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	int nPoints, nDim;
	file.open(filename.c_str(), ifstream::in);
	file >> nPoints >> nDim;

	for (int j = 0; j < nPoints; j++)
	{
		SIFT_Descriptor descriptor;
		double x, y, scale, direction, dummy;
		file >> y >> x >> scale >> direction;
		vector<double> desc;
		for (int iDim = 0; iDim < nDim; iDim++)
		{
			file >> dummy;
			desc.push_back(dummy);
		}
		descriptor.x = x;
		descriptor.y = y;
		descriptor.dis_x = x;
		descriptor.dis_y = y;
		descriptor.vDesc = desc;
		descriptor.id = j;
		descriptor.scale = scale;
		descriptor.direction = direction;

		vDescriptor.push_back(descriptor);
	}
	file.close();
}


void LoadStaticMeasurementData(string filename, CvMat *K, double k1, double k2, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras, vector<Feature> &vUncalibratedFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nFrames;
	file >> a >> nCameras;
	file >> a >> nFeatures;
	nFeatures = nFeatures;

	double xc = cvGetReal2D(K, 0, 2);
	double yc = cvGetReal2D(K, 1, 2);
	CvMat *invK = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K, invK);
	double ik11 = cvGetReal2D(invK, 0, 0);	double ik12 = cvGetReal2D(invK, 0, 1);	double ik13 = cvGetReal2D(invK, 0, 2);
	double ik21 = cvGetReal2D(invK, 1, 0);	double ik22 = cvGetReal2D(invK, 1, 1);	double ik23 = cvGetReal2D(invK, 1, 2);
	double ik31 = cvGetReal2D(invK, 2, 0);	double ik32 = cvGetReal2D(invK, 2, 1);	double ik33 = cvGetReal2D(invK, 2, 2);
	double k11 = cvGetReal2D(K, 0, 0);	double k12 = cvGetReal2D(K, 0, 1);	double k13 = cvGetReal2D(K, 0, 2);
	double k21 = cvGetReal2D(K, 1, 0);	double k22 = cvGetReal2D(K, 1, 1);	double k23 = cvGetReal2D(K, 1, 2);
	double k31 = cvGetReal2D(K, 2, 0);	double k32 = cvGetReal2D(K, 2, 1);	double k33 = cvGetReal2D(K, 2, 2);
	double nzc = (ik31*xc+ik32*yc+ik33);
	double nxc = (ik11*xc+ik12*yc+ik13)/nzc; 
	double nyc = (ik21*xc+ik22*yc+ik23)/nzc; 

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		int id;
		file >> nVisibleFrame >> id;

		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0;
		double x, y;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int frame, cam;			
			double L, r;
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y;
			x0 = x;
			y0 = y;
			
			double nz = (ik31*x+ik32*y+ik33);
			double nx = (ik11*x+ik12*y+ik13)/nz; 
			double ny = (ik21*x+ik22*y+ik23)/nz; 

			r = sqrt((nx-nxc)*(nx-nxc)+(ny-nyc)*(ny-nyc));
			L = 1 + k1*r + k2*r*r;
			nx = nxc + L*(nx - nxc);
			ny = nyc + L*(ny - nyc);

			double z = (k31*nx+k32*ny+k33);
			x = (k11*nx+k12*ny+k13)/z; 
			y = (k21*nx+k22*ny+k23)/z; 

			vCam.push_back(cam);
			vFrame.push_back(frame+cam*nFrames);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vFrame0.push_back(frame);
		}
		//if (vFrame.size() < 5)
		//	continue;
		feature.id = id;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		//feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;
		
		vUncalibratedFeature.push_back(feature);
	}
	//vFeature.pop_back();
	file.close();
}

void LoadStaticMeasurementData(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras, nFeatures;
	file >> a >> nCameras;
	file >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vCamera[iCamera].nFrames = nFrames;
	}
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			if (vCamera[iCamera].vTakenFrame[iFrame] > max_nFrames)
				max_nFrames = vCamera[iCamera].vTakenFrame[iFrame];
		}
	}
	max_nFrames++;
	file >> a >> nFeatures;

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		int id;
		file >> nVisibleFrame >> id;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0;
		double x, y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double L, r;
			double x0,y0;
			file >> cam >> frame;
			CvMat *K = cvCreateMat(3,3,CV_32FC1);
			CvMat *invK = cvCreateMat(3,3,CV_32FC1);
			int idx = -1;
			for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
			{
				if (vCamera[iCamera].id == cam)
				{
					idx = iCamera;
					break;
				}
			}
			K = cvCloneMat(vCamera[idx].K);
			double k1 = vCamera[idx].k1;
			double k2 = vCamera[idx].k2;

			double xc = cvGetReal2D(K, 0, 2);
			double yc = cvGetReal2D(K, 1, 2);
			
			cvInvert(K, invK);
			double ik11 = cvGetReal2D(invK, 0, 0);	double ik12 = cvGetReal2D(invK, 0, 1);	double ik13 = cvGetReal2D(invK, 0, 2);
			double ik21 = cvGetReal2D(invK, 1, 0);	double ik22 = cvGetReal2D(invK, 1, 1);	double ik23 = cvGetReal2D(invK, 1, 2);
			double ik31 = cvGetReal2D(invK, 2, 0);	double ik32 = cvGetReal2D(invK, 2, 1);	double ik33 = cvGetReal2D(invK, 2, 2);
			double k11 = cvGetReal2D(K, 0, 0);	double k12 = cvGetReal2D(K, 0, 1);	double k13 = cvGetReal2D(K, 0, 2);
			double k21 = cvGetReal2D(K, 1, 0);	double k22 = cvGetReal2D(K, 1, 1);	double k23 = cvGetReal2D(K, 1, 2);
			double k31 = cvGetReal2D(K, 2, 0);	double k32 = cvGetReal2D(K, 2, 1);	double k33 = cvGetReal2D(K, 2, 2);
			double nzc = (ik31*xc+ik32*yc+ik33);
			double nxc = (ik11*xc+ik12*yc+ik13)/nzc; 
			double nyc = (ik21*xc+ik22*yc+ik23)/nzc; 
			file >> x >> y;
			x0 = x;
			y0 = y;

			double nz = (ik31*x+ik32*y+ik33);
			double nx = (ik11*x+ik12*y+ik13)/nz; 
			double ny = (ik21*x+ik22*y+ik23)/nz; 

			r = sqrt((nx-nxc)*(nx-nxc)+(ny-nyc)*(ny-nyc));
			L = 1 + k1*r + k2*r*r;
			nx = nxc + L*(nx - nxc);
			ny = nyc + L*(ny - nyc);

			double z = (k31*nx+k32*ny+k33);
			x = (k11*nx+k12*ny+k13)/z; 
			y = (k21*nx+k22*ny+k23)/z; 

			vCam.push_back(cam);

			vFrame.push_back(vCamera[idx].vTakenFrame[frame]+idx*max_nFrames);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vFrame0.push_back(frame);
			
			cvReleaseMat(&K);
			cvReleaseMat(&invK);
		}
		//if (vFrame.size() < 5)
		//	continue;
		feature.id = id;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;

		vUncalibratedFeature.push_back(feature);
	}
	file.close();
}

void LoadStaticMeasurementData_NoCalibration(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras, nFeatures;
	file >> a >> nCameras;
	file >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vCamera[iCamera].nFrames = nFrames;
	}
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			if (vCamera[iCamera].vTakenFrame[iFrame] > max_nFrames)
				max_nFrames = vCamera[iCamera].vTakenFrame[iFrame];
		}
	}
	max_nFrames++;
	file >> a >> nFeatures;

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0;
		double x, y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y;
			x0 = x;
			y0 = y;

			int idx = -1;
			for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
			{
				if (vCamera[iCamera].id == cam)
				{
					idx = iCamera;
					break;
				}
			}

			if ((idx == -1) || (vCamera[idx].vTakenFrame.size() <= frame))
				continue;
			vCam.push_back(cam);
			vFrame.push_back(vCamera[idx].vTakenFrame[frame]+idx*max_nFrames);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vFrame0.push_back(frame);
		}
		//if (vFrame.size() < 5)
		//	continue;
		feature.id = id;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;

		vUncalibratedFeature.push_back(feature);
	}
	file.close();
}

void LoadStaticMeasurementData_NoCalibration_RGB(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras, nFeatures;
	file >> a >> nCameras;
	file >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vCamera[iCamera].nFrames = vCamera[iCamera].vTakenFrame.size();
	}
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			if (vCamera[iCamera].vTakenFrame[iFrame] > max_nFrames)
				max_nFrames = vCamera[iCamera].vTakenFrame[iFrame];
		}
	}
	max_nFrames++;
	file >> a >> nFeatures;

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >> b;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0;
		double x, y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y;
			x0 = x;
			y0 = y;

			int idx = -1;
			for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
			{
				if (vCamera[iCamera].id == cam)
				{
					idx = iCamera;
					break;
				}
			}

			if ((idx == -1) || (vCamera[idx].vTakenFrame.size() <= frame))
				continue;
			vCam.push_back(cam);
			vFrame.push_back(vCamera[idx].vTakenFrame[frame]+idx*max_nFrames);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vFrame0.push_back(frame);
		}
		//if (vFrame.size() < 5)
		//	continue;
		feature.id = id;
		feature.r = r; 
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;

		vUncalibratedFeature.push_back(feature);
	}
	file.close();
}

void LoadStaticMeasurementData_NoCalibration_RGB_LOWES(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras, nFeatures;
	file >> a >> nCameras;
	file >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vCamera[iCamera].nFrames = vCamera[iCamera].vTakenFrame.size();
	}
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			if (vCamera[iCamera].vTakenFrame[iFrame] > max_nFrames)
				max_nFrames = vCamera[iCamera].vTakenFrame[iFrame];
		}
	}
	max_nFrames++;
	file >> a >> nFeatures;

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >> b;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0;
		double x, y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> y >> x;
			x0 = x;
			y0 = y;

			int idx = -1;
			for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
			{
				if (vCamera[iCamera].id == cam)
				{
					idx = iCamera;
					break;
				}
			}

			if ((idx == -1) || (vCamera[idx].vTakenFrame.size() <= frame))
				continue;
			vCam.push_back(cam);
			vFrame.push_back(vCamera[idx].vTakenFrame[frame]+idx*max_nFrames);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vFrame0.push_back(frame);
		}
		//if (vFrame.size() < 5)
		//	continue;
		feature.id = id;
		feature.r = r; 
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;

		vUncalibratedFeature.push_back(feature);
	}
	file.close();
}

void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras, nFeatures;
	file >> a >> nCameras;
	file >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vCamera[iCamera].nFrames = vCamera[iCamera].vTakenFrame.size();
	}
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			if (vCamera[iCamera].vTakenFrame[iFrame] > max_nFrames)
				max_nFrames = vCamera[iCamera].vTakenFrame[iFrame];
		}
	}
	max_nFrames++;
	file >> a >> nFeatures;

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >> b;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0;
		double x, y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y;
			x0 = x;
			y0 = y;

			int idx = -1;
			for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
			{
				if (vCamera[iCamera].id == cam)
				{
					idx = iCamera;
					break;
				}
			}

			if ((idx == -1) || (vCamera[idx].vTakenFrame.size() <= frame))
				continue;
			vCam.push_back(cam);
			vFrame.push_back(vCamera[idx].vTakenFrame[frame]+idx*max_nFrames);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vFrame0.push_back(frame);
		}
		//if (vFrame.size() < 5)
		//	continue;
		feature.id = id;
		feature.r = r; 
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		feature.isRegistered = false;

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;

		vUncalibratedFeature.push_back(feature);
	}
	file.close();
}

void LoadStaticMeasurementData_NoCalibration_RGB(string filename, vector<Camera> &vCamera, vector<Camera> &vCameraTemp, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras, nFeatures;
	file >> a >> nCameras;
	file >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vCamera[iCamera].nFrames = vCamera[iCamera].vTakenFrame.size();
	}
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			if (vCamera[iCamera].vTakenFrame[iFrame] > max_nFrames)
				max_nFrames = vCamera[iCamera].vTakenFrame[iFrame];
		}
	}
	max_nFrames++;
	file >> a >> nFeatures;

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >> b;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0;
		double x, y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y;
			x0 = x;
			y0 = y;

			int idx = -1;
			for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
			{
				if (vCamera[iCamera].id == cam)
				{
					idx = iCamera;
					break;
				}
			}

			
			
			if ((idx == -1) || (vCamera[idx].vTakenFrame.size() <= frame))
				continue;
			vector<int>::iterator it = find(vCameraTemp[idx].vTakenFrame.begin(), vCameraTemp[idx].vTakenFrame.end(), vCamera[idx].vTakenFrame[frame]);
			if (it == vCameraTemp[idx].vTakenFrame.end())
				continue;
			vCam.push_back(cam);
			vFrame.push_back(vCamera[idx].vTakenFrame[frame]+idx*max_nFrames);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vFrame0.push_back(frame);
		}
		//if (vFrame.size() < 5)
		//	continue;
		feature.id = id;
		feature.r = r; 
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;

		vUncalibratedFeature.push_back(feature);
	}
	file.close();
}




void LoadStaticMeasurementData(string filename, CvMat *K, double k1, double k2, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nFrames;
	file >> a >> nCameras;
	file >> a >> nFeatures;
	nFeatures = nFeatures;

	double xc = cvGetReal2D(K, 0, 2);
	double yc = cvGetReal2D(K, 1, 2);
	CvMat *invK = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K, invK);
	double ik11 = cvGetReal2D(invK, 0, 0);	double ik12 = cvGetReal2D(invK, 0, 1);	double ik13 = cvGetReal2D(invK, 0, 2);
	double ik21 = cvGetReal2D(invK, 1, 0);	double ik22 = cvGetReal2D(invK, 1, 1);	double ik23 = cvGetReal2D(invK, 1, 2);
	double ik31 = cvGetReal2D(invK, 2, 0);	double ik32 = cvGetReal2D(invK, 2, 1);	double ik33 = cvGetReal2D(invK, 2, 2);
	double k11 = cvGetReal2D(K, 0, 0);	double k12 = cvGetReal2D(K, 0, 1);	double k13 = cvGetReal2D(K, 0, 2);
	double k21 = cvGetReal2D(K, 1, 0);	double k22 = cvGetReal2D(K, 1, 1);	double k23 = cvGetReal2D(K, 1, 2);
	double k31 = cvGetReal2D(K, 2, 0);	double k32 = cvGetReal2D(K, 2, 1);	double k33 = cvGetReal2D(K, 2, 2);
	double nzc = (ik31*xc+ik32*yc+ik33);
	double nxc = (ik11*xc+ik12*yc+ik13)/nzc; 
	double nyc = (ik21*xc+ik22*yc+ik23)/nzc; 

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		int id;
		file >> nVisibleFrame >> id;

		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;
		double x, y;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int frame, cam;			
			double L, r;
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y;

			double nz = (ik31*x+ik32*y+ik33);
			double nx = (ik11*x+ik12*y+ik13)/nz; 
			double ny = (ik21*x+ik22*y+ik23)/nz; 

			r = sqrt((nx-nxc)*(nx-nxc)+(ny-nyc)*(ny-nyc));
			L = 1 + k1*r + k2*r*r;
			nx = nxc + L*(nx - nxc);
			ny = nyc + L*(ny - nyc);

			double z = (k31*nx+k32*ny+k33);
			x = (k11*nx+k12*ny+k13)/z; 
			y = (k21*nx+k22*ny+k23)/z; 

			vCam.push_back(cam);
			vFrame.push_back(frame+cam*nFrames);
			vx.push_back(x);	vy.push_back(y);
		}
		//if (vFrame.size() < 5)
		//	continue;
		feature.id = id;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		//feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
	}
	//vFeature.pop_back();
	file.close();
}



void LoadMeasurementData(string filename, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nCameras;
	file >> a >> nFrames;
	file >> a >> nFeatures;
	int featureId = 0;
	//while (!file.eof())
	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame, id;
		file >> nVisibleFrame >> id;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;
			
			file >> cam >> frame >> x >> y;
			//vFrame.push_back(frame);
			vFrame.push_back(frame+nFrames*cam);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
		}
		//if (vFrame.size() < 8)
		//	continue;
		feature.id = featureId++;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		//if (d == "D")
		//	feature.isInlier.push_back(FEATURE_PROPERTY_DYNAMIC);
		//else
		//	feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
	}
	//vFeature.pop_back();
	file.close();
}

void LoadMeasurementData(string filename, vector<Feature> &vFeature, int &nFeatures, vector<int> &vnFrames, int &nCameras, int &max_nFrames)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nCameras;
	file >> a;
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vnFrames.push_back(nFrames);
		if (nFrames > max_nFrames)
			max_nFrames = nFrames;
	}
	max_nFrames++;
	file >> a >> nFeatures;
	int featureId = 0;
	//while (!file.eof())
	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame, id;
		file >> nVisibleFrame >> id;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;

			file >> cam >> frame >> x >> y;
			//vFrame.push_back(frame);
			vFrame.push_back(frame+max_nFrames*cam);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
		}
		//if (vFrame.size() < 8)
		//	continue;
		feature.id = featureId++;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		//if (d == "D")
		//	feature.isInlier.push_back(FEATURE_PROPERTY_DYNAMIC);
		//else
		//	feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
	}
	//vFeature.pop_back();
	file.close();
}

void LoadMeasurementData(string filename, vector<Feature> &vFeature, int &nFeatures, vector<int> &vnFrames, int &nCameras, int &max_nFrames, int &max_nCameras)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nCameras;
	file >> a;
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vnFrames.push_back(nFrames);
		if (nFrames > max_nFrames)
			max_nFrames = nFrames;
	}
	max_nFrames++;
	file >> a >> nFeatures;
	int featureId = 0;
	//while (!file.eof())
	max_nCameras = 0;
	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame, id;
		file >> nVisibleFrame >> id;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;

			file >> cam >> frame >> x >> y;
			//vFrame.push_back(frame);
			vFrame.push_back(frame+max_nFrames*cam);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			if (cam > max_nCameras)
				max_nCameras = cam;
		}
		//if (vFrame.size() < 8)
		//	continue;
		feature.id = featureId++;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		//if (d == "D")
		//	feature.isInlier.push_back(FEATURE_PROPERTY_DYNAMIC);
		//else
		//	feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
	}
	//vFeature.pop_back();
	file.close();
	max_nCameras++;
}

void LoadMeasurementData_RGB(string filename, vector<Feature> &vFeature, int &nFeatures, vector<int> &vnFrames, int &nCameras, int &max_nFrames, int &max_nCameras)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nCameras;
	file >> a;
	max_nFrames = 0;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		int nFrames;
		file >> nFrames;
		vnFrames.push_back(nFrames);
		if (nFrames > max_nFrames)
			max_nFrames = nFrames;
	}
	max_nFrames++;
	file >> a >> nFeatures;
	int featureId = 0;
	//while (!file.eof())
	max_nCameras = 0;
	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame, id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >>  b;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;

			file >> cam >> frame >> x >> y;
			//vFrame.push_back(frame);
			vFrame.push_back(frame+max_nFrames*cam);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			if (cam > max_nCameras)
				max_nCameras = cam;
		}
		//if (vFrame.size() < 8)
		//	continue;
		feature.id = featureId++;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		//if (d == "D")
		//	feature.isInlier.push_back(FEATURE_PROPERTY_DYNAMIC);
		//else
		//	feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
	}
	//vFeature.pop_back();
	file.close();
	max_nCameras++;
}

void LoadMeasurementData_RGB_DESC(string filename, vector<Feature> &vFeature, int &nFeatures, vector<int> &vnFrames, int &nCameras, int &max_nFrames, int &max_nCameras)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	//file >> a >> nCameras;
	//file >> a;
	//max_nFrames = 0;
	//for (int iCamera = 0; iCamera < nCameras; iCamera++)
	//{
	//	int nFrames;
	//	file >> nFrames;
	//	vnFrames.push_back(nFrames);
	//	if (nFrames > max_nFrames)
	//		max_nFrames = nFrames;
	//}
	//max_nFrames++;
	//file >> a >> nFeatures;
	int featureId = 0;
	max_nCameras = 1;
	nCameras = 1;
	max_nFrames = 0;
	while (!file.eof())	
	//for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame, id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >>  b;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;
		vector<vector<double> > vvDesc;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;

			file >> cam >> frame >> x >> y;

			vector<double> vDesc;
			for (int i = 0; i < 128; i++)
			{
				int dummy_int;
				file >> dummy_int;
				vDesc.push_back(dummy_int);
			}

			//vFrame.push_back(frame);
			vFrame.push_back(frame+max_nFrames*cam);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			vvDesc.push_back(vDesc);
			if (cam > max_nCameras)
				max_nCameras = cam;
			if (frame > max_nFrames)
				max_nFrames = frame;
		}
		//if (vFrame.size() < 8)
		//	continue;
		feature.id = featureId++;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		vector<double> vMeanDesc;
		for (int i = 0; i < 128; i++)
		{
			int mean = 0;
			for (int j = 0; j < vvDesc.size(); j++)
			{
				mean += vvDesc[j][i];
			}
			vMeanDesc.push_back((int) ((double)mean)/((double)vvDesc.size()));
		}
		vMeanDesc.push_back(vvDesc.size());
		vvDesc.clear();
		vvDesc.push_back(vMeanDesc);

		feature.vvDesc = vvDesc;
		//if (d == "D")
		//	feature.isInlier.push_back(FEATURE_PROPERTY_DYNAMIC);
		//else
		//	feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
	}
	vFeature.pop_back();
	vnFrames.push_back(max_nFrames);
	//vFeature.pop_back();
	file.close();
	max_nCameras++;
	max_nFrames++;
}

void LoadMeasurementData_RGB_Seq(string filename, vector<Feature> &vFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int featureId = 0;
	if (vFeature.size() == 0)
		featureId = 0;
	else
		featureId = vFeature[vFeature.size()-1].id + 1;
	while (!file.eof())	
	{
		int nVisibleFrame, id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >>  b;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;
		vector<double> vx_dis, vy_dis;
		vector<vector<double> > vvDesc;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;
			double x_dis, y_dis;
			file >> cam >> frame >> x >> y >> x_dis >> y_dis;

			//vector<double> vDesc;
			//for (int i = 0; i < 128; i++)
			//{
			//	int dummy_int;
			//	file >> dummy_int;
			//	vDesc.push_back(dummy_int);
			//}

			//vFrame.push_back(frame);
			vFrame.push_back(frame);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			vx_dis.push_back(x_dis); vy_dis.push_back(y_dis);
			//vvDesc.push_back(vDesc);
		}

		feature.id = featureId++;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		//vector<int> vMeanDesc;
		//for (int i = 0; i < 128; i++)
		//{
		//	int mean = 0;
		//	for (int j = 0; j < vvDesc.size(); j++)
		//	{
		//		mean += vvDesc[j][i];
		//	}
		//	vMeanDesc.push_back((int) ((double)mean)/((double)vvDesc.size()));
		//}
		//vMeanDesc.push_back(vvDesc.size());
		//vvDesc.clear();
		//vvDesc.push_back(vMeanDesc);

		//feature.vvDesc = vvDesc;
		vFeature.push_back(feature);
	}
	vFeature.pop_back();
	file.close();
}

void LoadMeasurementData_RGB_DESC_Seq(string filename, vector<Feature> &vFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int featureId = 0;
	if (vFeature.size() == 0)
		featureId = 0;
	else
		featureId = vFeature[vFeature.size()-1].id + 1;
	while (!file.eof())	
	{
		int nVisibleFrame, id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >>  b;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;
		vector<double> vx_dis, vy_dis;
		vector<vector<double> > vvDesc;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;
			double x_dis, y_dis;
			file >> cam >> frame >> x >> y >> x_dis >> y_dis;

			vector<double> vDesc;
			for (int i = 0; i < 128; i++)
			{
				int dummy_int;
				file >> dummy_int;
				vDesc.push_back(dummy_int);
			}

			//vFrame.push_back(frame);
			vFrame.push_back(frame);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			vx_dis.push_back(x_dis); vy_dis.push_back(y_dis);
			vvDesc.push_back(vDesc);
		}

		feature.id = featureId++;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		vector<double> vMeanDesc;
		for (int i = 0; i < 128; i++)
		{
			int mean = 0;
			for (int j = 0; j < vvDesc.size(); j++)
			{
				mean += vvDesc[j][i];
			}
			vMeanDesc.push_back((int) ((double)mean)/((double)vvDesc.size()));
		}
		vMeanDesc.push_back(vvDesc.size());
		//feature.vvDesc_AllDescriptors = vvDesc;
		vvDesc.clear();
		vvDesc.push_back(vMeanDesc);

		feature.vvDesc = vvDesc;
		vFeature.push_back(feature);
	}
	vFeature.pop_back();
	file.close();
}

void LoadMeasurementData_RGB_DESC_Seq_SURF(string filename, vector<Feature> &vFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int featureId = 0;
	if (vFeature.size() == 0)
		featureId = 0;
	else
		featureId = vFeature[vFeature.size()-1].id + 1;
	while (!file.eof())	
	{
		int nVisibleFrame, id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >>  b;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;
		vector<double> vx_dis, vy_dis;
		vector<vector<double> > vvDesc;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;
			double x_dis, y_dis;
			file >> cam >> frame >> x >> y >> x_dis >> y_dis;

			vector<double> vDesc;
			for (int i = 0; i < 64; i++)
			{
				double dummy_int;
				file >> dummy_int;
				vDesc.push_back(dummy_int);
			}

			//vFrame.push_back(frame);
			vFrame.push_back(frame);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			vx_dis.push_back(x_dis); vy_dis.push_back(y_dis);
			vvDesc.push_back(vDesc);
		}

		feature.id = featureId++;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		vector<double> vMeanDesc;
		for (int i = 0; i < 64; i++)
		{
			double mean = 0;
			for (int j = 0; j < vvDesc.size(); j++)
			{
				mean += vvDesc[j][i];
			}
			vMeanDesc.push_back(((double)mean)/((double)vvDesc.size()));
		}
		vMeanDesc.push_back(vvDesc.size());
		//feature.vvDesc_AllDescriptors = vvDesc;
		vvDesc.clear();
		vvDesc.push_back(vMeanDesc);

		feature.vvDesc = vvDesc;
		vFeature.push_back(feature);
	}
	vFeature.pop_back();
	file.close();
}

void LoadMeasurementData_RGB_DESC_Seq_ScaleDirection(string filename, vector<Feature> &vFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int featureId = 0;
	if (vFeature.size() == 0)
		featureId = 0;
	else
		featureId = vFeature[vFeature.size()-1].id + 1;
	while (!file.eof())	
	{
		int nVisibleFrame, id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >>  b;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;
		vector<double> vx_dis, vy_dis;
		vector<vector<double> > vvDesc;
		vector<double> vScale, vDirection;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;
			double x_dis, y_dis;
			file >> cam >> frame >> x >> y >> x_dis >> y_dis;

			double scale, direction;
			file >> scale >> direction;

			vector<double> vDesc;
			for (int i = 0; i < 128; i++)
			{
				int dummy_int;
				file >> dummy_int;
				vDesc.push_back(dummy_int);
			}

			//vFrame.push_back(frame);
			vFrame.push_back(frame);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			vx_dis.push_back(x_dis); vy_dis.push_back(y_dis);
			vvDesc.push_back(vDesc);
			vScale.push_back(scale);
			vDirection.push_back(direction);
		}

		feature.id = featureId++;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		feature.vDirection = vDirection;
		feature.vScale = vScale;
		vector<double> vMeanDesc;
		for (int i = 0; i < 128; i++)
		{
			int mean = 0;
			for (int j = 0; j < vvDesc.size(); j++)
			{
				mean += vvDesc[j][i];
			}
			vMeanDesc.push_back((int) ((double)mean)/((double)vvDesc.size()));
		}
		vMeanDesc.push_back(vvDesc.size());
		feature.vvDesc_AllDescriptors = vvDesc;
		vvDesc.clear();
		vvDesc.push_back(vMeanDesc);

		feature.vvDesc = vvDesc;
		vFeature.push_back(feature);
	}
	vFeature.pop_back();
	file.close();
}

void LoadMeasurementData_RGB_DESC_Seq1(string filename, vector<Feature> &vFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int featureId = 0;
	if (vFeature.size() == 0)
		featureId = 0;
	else
		featureId = vFeature[vFeature.size()-1].id + 1;
	while (!file.eof())	
	{
		int nVisibleFrame, id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >>  b;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;
		vector<double> vx_dis, vy_dis;
		//vector<vector<double> > vvDesc;

		vector<double> vDesc;
		for (int i = 0; i < 128; i++)
		{
			int dummy_int;
			file >> dummy_int;
			vDesc.push_back(dummy_int);
		}

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;
			double x_dis, y_dis;
			file >> cam >> frame;
				
		    file >> x >> y >> x_dis >> y_dis;
			//vFrame.push_back(frame);
			vFrame.push_back(frame);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			vx_dis.push_back(x_dis); vy_dis.push_back(y_dis);
			//vvDesc.push_back(vDesc);
		}

		feature.id = featureId++;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		vector<double> vMeanDesc = vDesc;
		vMeanDesc.push_back(vx.size());

		vector<vector<double> > vvDesc;
		vvDesc.push_back(vMeanDesc);

		feature.vvDesc = vvDesc;
		vFeature.push_back(feature);
	}
	vFeature.pop_back();
	file.close();
}

void LoadMeasurementData_RGB_DESC_Seq_Interpolation(string filename, vector<Feature> &vFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int featureId = 0;
	if (vFeature.size() == 0)
		featureId = 0;
	else
		featureId = vFeature[vFeature.size()-1].id + 1;
	while (!file.eof())	
	{
		int nVisibleFrame, id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >>  b;
		Feature feature;	
		vector<int> vFrame, vCam;
		vector<double> vx, vy;
		vector<double> vx_dis, vy_dis;
		vector<vector<double> > vvDesc;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;
			double x_dis, y_dis;
			file >> cam >> frame >> x >> y >> x_dis >> y_dis;

			//vFrame.push_back(frame);
			vFrame.push_back(frame);
			vCam.push_back(cam);
			vx.push_back(x);	vy.push_back(y);
			vx_dis.push_back(x_dis); vy_dis.push_back(y_dis);
		}

		feature.id = featureId++;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		vFeature.push_back(feature);
	}
	vFeature.pop_back();
	file.close();
}

void LoadMeasurementData(string filename, CvMat *K, double k1, double k2, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nFrames;
	file >> a >> nCameras;
	file >> a >> nFeatures;
	int featureId = 0;
	//while (!file.eof())
	
	double xc = cvGetReal2D(K, 0, 2);
	double yc = cvGetReal2D(K, 1, 2);
	CvMat *invK = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K, invK);
	double ik11 = cvGetReal2D(invK, 0, 0);	double ik12 = cvGetReal2D(invK, 0, 1);	double ik13 = cvGetReal2D(invK, 0, 2);
	double ik21 = cvGetReal2D(invK, 1, 0);	double ik22 = cvGetReal2D(invK, 1, 1);	double ik23 = cvGetReal2D(invK, 1, 2);
	double ik31 = cvGetReal2D(invK, 2, 0);	double ik32 = cvGetReal2D(invK, 2, 1);	double ik33 = cvGetReal2D(invK, 2, 2);
	double k11 = cvGetReal2D(K, 0, 0);	double k12 = cvGetReal2D(K, 0, 1);	double k13 = cvGetReal2D(K, 0, 2);
	double k21 = cvGetReal2D(K, 1, 0);	double k22 = cvGetReal2D(K, 1, 1);	double k23 = cvGetReal2D(K, 1, 2);
	double k31 = cvGetReal2D(K, 2, 0);	double k32 = cvGetReal2D(K, 2, 1);	double k33 = cvGetReal2D(K, 2, 2);
	double nzc = (ik31*xc+ik32*yc+ik33);
	double nxc = (ik11*xc+ik12*yc+ik13)/nzc; 
	double nyc = (ik21*xc+ik22*yc+ik23)/nzc; 

	for (int j = 0; j < nFeatures; j++)
	{
		int nVisibleFrame;
		string d;
		file >> d;
		file >> nVisibleFrame;

		Feature feature;	
		vector<int> vFrame;
		vector<double> vx, vy;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame;
			double x, y;
			double L, r;
			file >> frame >> x >> y >> temp;
			double nz = (ik31*x+ik32*y+ik33);
			double nx = (ik11*x+ik12*y+ik13)/nz; 
			double ny = (ik21*x+ik22*y+ik23)/nz; 

			r = sqrt((nx-nxc)*(nx-nxc)+(ny-nyc)*(ny-nyc));
			L = 1 + k1*r + k2*r*r;
			nx = nxc + L*(nx - nxc);
			ny = nyc + L*(ny - nyc);

			double z = (k31*nx+k32*ny+k33);
			x = (k11*nx+k12*ny+k13)/z; 
			y = (k21*nx+k22*ny+k23)/z; 

			vFrame.push_back(frame);
			vx.push_back(x);	vy.push_back(y);
		}
		//if (vFrame.size() < 3)
		//	continue;
		feature.id = featureId++;
		feature.vFrame = vFrame;
		feature.vx = vx;
		feature.vy = vy;
		//if (d == "D")
		//	feature.isInlier.push_back(FEATURE_PROPERTY_DYNAMIC);
		//else
		//	feature.isInlier.push_back(FEATURE_PROPERTY_STATIC);

		vFeature.push_back(feature);
	}
	//vFeature.pop_back();
	file.close();
}

void LoadThetaData(string filename, vector<Theta> &vTheta, int &nFeatures, int &nFrames, int &nCameras, int &nBase)
{
	PrintAlgorithm("Load Theta Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> nFrames;
	file >> a >> nCameras;
	file >> a >> nFeatures;
	file >> a >> nBase;
	int featureId = 0;
	for (int j = 0; j < nFeatures; j++)
	{
		Theta theta;
		vector<double> vX, vY, vZ;
		string d;
		file >> d;
		file >> featureId;
		for (int i = 0; i < nBase; i++)
		{
			double t;
			file >> t;
			vX.push_back(t);
		}
		for (int i = 0; i < nBase; i++)
		{
			double t;
			file >> t;
			vY.push_back(t);
		}
		for (int i = 0; i < nBase; i++)
		{
			double t;
			file >> t;
			vZ.push_back(t);
		}
		theta.id = featureId;
		theta.thetaX = vX;
		theta.thetaY = vY;
		theta.thetaZ = vZ;
		if (d == "S")
			theta.isStatic = 1;
		else
			theta.isStatic = 0;
		vTheta.push_back(theta);
	}
	file.close();
}

void LoadInitialFileData(string filename, int &frame1, int &frame2, int &nFrames)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	fin >> frame1 >> frame2 >> nFrames;
	fin.close();
}

void LoadCameraData(string filename, vector<Camera> &vCamera, CvMat *K)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a >> nFrames;
	fin >> a >> nCameras;
	fin >> a >> nTotalFrames;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *t = cvCreateMat(3,1,CV_32FC1);
	CvMat *P;
	for (int i = 0; i < nTotalFrames; i++)
	{
		Camera camera;
		double iCamera, iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33;
		fin >> iCamera >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		cvSetReal2D(t, 0,0, -t1);	cvSetReal2D(t, 1,0, -t2);	cvSetReal2D(t, 2,0, -t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);
		P = cvCreateMat(3,4,CV_32FC1);
		cvSetIdentity(P);
		SetSubMat(P, 0, 3, t);
		cvMatMul(R, P, P);
		cvMatMul(K, P, P);
		
		bool isCamera = false;
		for (int iCam = 0; iCam < vCamera.size(); iCam++)
		{
			if (vCamera[iCam].id == iCamera)
			{
				vCamera[iCam].vP.push_back(P);
				vCamera[iCam].vTakenFrame.push_back(iFrame);
				isCamera = true;
				break;
			}
		}
		if (!isCamera)
		{
			camera.id = iCamera;
			camera.vP.push_back(P);
			camera.vTakenFrame.push_back(iFrame);
			vCamera.push_back(camera);
		}
	}	

	fin.close();

	cvReleaseMat(&R);
	cvReleaseMat(&t);
}

void LoadCameraData(string filename, vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)	
	{
		vCamera[iCamera].id = iCamera;
		vCamera[iCamera].vTakenFrame.clear();
		fin >> nFrames;
	}
	fin >> a >> nTotalFrames;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *t = cvCreateMat(3,1,CV_32FC1);
	CvMat *P;
	for (int i = 0; i < nTotalFrames; i++)
	{
		Camera camera;
		double iCamera, iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33;
		fin >> iCamera >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		cvSetReal2D(t, 0,0, -t1);	cvSetReal2D(t, 1,0, -t2);	cvSetReal2D(t, 2,0, -t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);
		P = cvCreateMat(3,4,CV_32FC1);
		cvSetIdentity(P);
		SetSubMat(P, 0, 3, t);
		cvMatMul(R, P, P);
		cvMatMul(vCamera[iCamera].K, P, P);

		bool isCamera = false;
		for (int iCam = 0; iCam < vCamera.size(); iCam++)
		{
			if (vCamera[iCam].id == iCamera)
			{
				vCamera[iCam].vP.push_back(P);
				vCamera[iCam].vTakenFrame.push_back(iFrame);
				CvMat *K = cvCreateMat(3,3,CV_32FC1);
				K = cvCloneMat(vCamera[iCam].K);
				vCamera[iCam].vK.push_back(K);
				vCamera[iCam].vTakenInstant.push_back(iFrame);
				isCamera = true;
				break;
			}
		}
		if (!isCamera)
		{
			camera.id = iCamera;
			camera.vP.push_back(P);
			camera.vTakenFrame.push_back(iFrame);
			vCamera.push_back(camera);
		}
	}	

	fin.close();

	cvReleaseMat(&R);
	cvReleaseMat(&t);
}

void LoadCameraData_TA(string filename, vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)	
	{
		fin >> nFrames;
	}
	fin >> a >> nTotalFrames;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *t = cvCreateMat(3,1,CV_32FC1);
	CvMat *P;
	for (int i = 0; i < nTotalFrames; i++)
	{
		Camera camera;
		double iCamera, iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33, timeInstant;
		int takenFrame;
		fin >> iCamera >> iFrame >> takenFrame >> timeInstant;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		cvSetReal2D(t, 0,0, -t1);	cvSetReal2D(t, 1,0, -t2);	cvSetReal2D(t, 2,0, -t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);
		//P = cvCreateMat(3,4,CV_32FC1);
		//cvSetIdentity(P);
		//SetSubMat(P, 0, 3, t);
		//cvMatMul(R, P, P);
		//cvMatMul(vCamera[iCamera].K, P, P);

		CvMat *C = cvCreateMat(3,1,CV_32FC1);
		CvMat *R_ = cvCreateMat(3,3,CV_32FC1);
		cvSetReal2D(C, 0, 0, t1);	cvSetReal2D(C, 1, 0, t2);	cvSetReal2D(C, 2, 0, t3);
		cvSetReal2D(R_, 0,0, r11);	cvSetReal2D(R_, 0,1, r12);	cvSetReal2D(R_, 0,2, r13);
		cvSetReal2D(R_, 1,0, r21);	cvSetReal2D(R_, 1,1, r22);	cvSetReal2D(R_, 1,2, r23);
		cvSetReal2D(R_, 2,0, r31);	cvSetReal2D(R_, 2,1, r32);	cvSetReal2D(R_, 2,2, r33);

		bool isCamera = false;
		for (int iCam = 0; iCam < vCamera.size(); iCam++)
		{
			if (vCamera[iCam].id == iCamera)
			{
				//vCamera[iCam].vP.push_back(P);
				vCamera[iCam].vTakenFrame.push_back(takenFrame);
				//CvMat *K = cvCreateMat(3,3,CV_32FC1);
				//K = cvCloneMat(vCamera[iCam].K);
				//vCamera[iCam].vK.push_back(K);
				vCamera[iCam].vC.push_back(C);
				vCamera[iCam].vR.push_back(R_);
				vCamera[iCam].vTakenInstant.push_back(timeInstant);

				isCamera = true;
				break;
			}
		}
		if (!isCamera)
		{
			camera.id = iCamera;
			//camera.vP.push_back(P);
			camera.vTakenFrame.push_back(takenFrame);
			camera.vTakenInstant.push_back(timeInstant);
			camera.vC.push_back(C);
			camera.vR.push_back(R_);
			vCamera.push_back(camera);
		}
	}	

	fin.close();

	cvReleaseMat(&R);
	cvReleaseMat(&t);
}

void LoadCameraData_POS(string filename, vector<double> &vx, vector<double> &vy, vector<double> &vz)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)	
	{
		fin >> nFrames;
	}
	fin >> a >> nTotalFrames;
	for (int i = 0; i < nTotalFrames; i++)
	{
		Camera camera;
		double iCamera, iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33;
		fin >> iCamera >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		vx.push_back(t1);
		vy.push_back(t2);
		vz.push_back(t3);
	}	

	fin.close();

}

void LoadCameraData_Add(string filename, vector<Camera> &vCamera, vector<CvMat *> &cP)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)	
	{
		fin >> nFrames;
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			CvMat *P = cvCreateMat(3,4,CV_32FC1);
			cvSetIdentity(P);
			vCamera[iCamera].vP.push_back(P);
		}
	}
	fin >> a >> nTotalFrames;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *t = cvCreateMat(3,1,CV_32FC1);
	for (int i = 0; i < nTotalFrames; i++)
	{
		CvMat *P;
		Camera camera;
		double iCamera, iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33;
		fin >> iCamera >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		cvSetReal2D(t, 0,0, -t1);	cvSetReal2D(t, 1,0, -t2);	cvSetReal2D(t, 2,0, -t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);
		P = cvCreateMat(3,4,CV_32FC1);
		cvSetIdentity(P);
		SetSubMat(P, 0, 3, t);
		cvMatMul(R, P, P);

		bool isCamera = false;
		for (int icam = 0; icam < vCamera.size(); icam++)
		{
			if ((vCamera[icam].id == iCamera) && (((int)vCamera[icam].vK.size()) > iFrame))
			{
				cvMatMul(vCamera[icam].vK[iFrame], P, P);
				cP.push_back(P);
				//PrintMat(P,"P");
				//vector<int>::const_iterator it = find(vCamera[icam].vTakenFrame.begin(), vCamera[icam].vTakenFrame.end(), iFrame);
				//int idx = (int) (it - vCamera[icam].vTakenFrame.begin());
				//cvMatMul(vCamera[icam].vK[idx], P, P);
				//cP.push_back(P);

				vCamera[icam].vP[iFrame] = cvCloneMat(P);
				isCamera = true;
				break;
			}
		}
		if (!isCamera)
			int q = 0;
	}	

	fin.close();

	cvReleaseMat(&R);
	cvReleaseMat(&t);
}

void LoadCameraIntrinsicData(string filename, vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	for (int iCamera = 0; iCamera < nCameras; iCamera++)	
	{
		fin >> nFrames;
	}
	fin >> a >> nTotalFrames;
	
	for (int i = 0; i < nTotalFrames; i++)
	{
		Camera camera;
		CvMat *K = cvCreateMat(3,3,CV_32FC1);
		double iCamera, iFrame, k11, k12, k13, k21, k22, k23, k31, k32, k33, k1, k2;
		fin >> iCamera >> iFrame;
		fin >> k11 >> k12 >> k13;
		fin >> k21 >> k22 >> k23;
		fin >> k31 >> k32 >> k33;
		fin >> k1 >> k2;
		cvSetReal2D(K, 0,0, k11);	cvSetReal2D(K, 0,1, k12);	cvSetReal2D(K, 0,2, k13);
		cvSetReal2D(K, 1,0, k21);	cvSetReal2D(K, 1,1, k22);	cvSetReal2D(K, 1,2, k23);
		cvSetReal2D(K, 2,0, k31);	cvSetReal2D(K, 2,1, k32);	cvSetReal2D(K, 2,2, k33);

		bool isCamera = false;
		for (int iCam = 0; iCam < vCamera.size(); iCam++)
		{
			if ((int)vCamera[iCam].vK.size() > iFrame)
			{
				//cvSetIdentity(vCamera[iCam].vK[iFrame]);
				if (vCamera[iCam].id == iCamera)
				{
					vCamera[iCam].vK[iFrame] = cvCloneMat(K);
					vCamera[iCam].vk1[iFrame] = k1;
					vCamera[iCam].vk2[iFrame] = k2;
					//vector<int>::const_iterator it = find(vCamera[iCam].vTakenFrame.begin(), vCamera[iCam].vTakenFrame.end(), iFrame);
					//int idx = (int) (it - vCamera[iCam].vTakenFrame.begin());
					//vCamera[iCam].vK[idx] = cvCloneMat(K);
					//vCamera[iCam].vk1[idx] = k1;
					//vCamera[iCam].vk2[idx] = k2;
					isCamera = true;
					break;
				}
			}

		}
		cvReleaseMat(&K);
	}	

	fin.close();
}

//void LoadCameraList(string filename, string path, vector<Camera> &vCamera)
//{
//	PrintAlgorithm("Load Camera List File");
//	cout << "File name: " << filename << endl;
//	ifstream fin;
//	fin.open(filename.c_str(), ifstream::in);
//	int nFrames, nCameras;
//	string dummy;
//	fin >> dummy >> nFrames;
//	fin >> dummy >> nCameras;
//
//	for (int iCamera = 0; iCamera < nCameras; iCamera++)
//	{
//		
//	}
//
//
//
//	int nFrames, nCameras, nTotalFrames;
//	string a;
//	while (!fin.eof())
//	{
//		int id, first, nDigit, nFrames;
//		string prefix, suffix, extension, folder;
//		fin >> id >> folder >> prefix >> suffix >> extension >> first >> nDigit >> nFrames;
//		if (strcmp(suffix.c_str(), "//"))
//			suffix = "";
//		Camera cam;
//		FileName fn, fn_d;
//		cam.id = id;		
//		fn.path = path + folder + "/";
//		fn.prefix = prefix;
//		fn.suffix = suffix;
//		fn.extension = extension;
//		fn.nDigit = nDigit;
//		fn_d = fn;
//		fn_d.prefix = "d" + fn.prefix;
//		cam.filename = fn;
//		cam.nFrames = nFrames;
//		cam.first = first;
//		cam.stride = 1;
//		cam.filename_d = fn_d;
//		for (int iTakenFrame = 0; iTakenFrame < cam.nFrames; iTakenFrame++)
//			cam.vTakenFrame.push_back(iTakenFrame);
//		vCamera.push_back(cam);
//	}
//	//vCamera.pop_back();
//	vCamera.pop_back();
//	fin.close();
//}

void LoadFileInfo(string filename, string path, vector<Camera> &vCamera)
{
	PrintAlgorithm("Load File Info");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	while (!fin.eof())
	{
		int id, first, nDigit, nFrames;
		string prefix, suffix, extension, folder;
		fin >> id >> folder >> prefix >> suffix >> extension >> first >> nDigit >> nFrames;
		if (strcmp(suffix.c_str(), "//"))
			suffix = "";
		Camera cam;
		FileName fn, fn_d;
		cam.id = id;		
		fn.path = path + folder + "/";
		fn.prefix = prefix;
		fn.suffix = suffix;
		fn.extension = extension;
		fn.nDigit = nDigit;
		fn_d = fn;
		fn_d.prefix = "d" + fn.prefix;
		cam.filename = fn;
		cam.nFrames = nFrames;
		cam.first = first;
		cam.stride = 1;
		cam.filename_d = fn_d;
		for (int iTakenFrame = 0; iTakenFrame < cam.nFrames; iTakenFrame++)
			cam.vTakenFrame.push_back(iTakenFrame);
		vCamera.push_back(cam);
	}
	//vCamera.pop_back();
	vCamera.pop_back();
	fin.close();
}

void LoadTimeOffset(string filename, vector<int> &vTimeoffset, vector<int> &vID)
{
	PrintAlgorithm("Load Time offset");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a;
	while (!fin.eof())
	{
		int id, hour, minute, second;
		fin >> id >> hour >> minute >> second;
		vID.push_back(id);
		vTimeoffset.push_back(hour*3600+minute*60+second);
	}
	//vCamera.pop_back();
	vID.pop_back();
	vTimeoffset.pop_back();
	fin.close();
}

void SaveCameraData(string filename, vector<CvMat*> cP, CvMat *K, int nFrames, int nCameras)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumFrames " << nFrames << endl;;
	fout << "NumCams " << nCameras << endl;
	fout << "NumP " << cP.size() << endl;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *t = cvCreateMat(3,1,CV_32FC1);
	CvMat *invK = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K, invK);
	CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
	CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
	CvMat *invR = cvCreateMat(3,3,CV_32FC1);
	CvMat *P = cvCreateMat(3,4,CV_32FC1);
	for (int i = 0; i < cP.size(); i++)
	{
		P = cvCloneMat(cP[i]);
		cvMatMul(invK, P, temp34);
		GetSubMatColwise(temp34, 0, 2, R);
		GetSubMatColwise(temp34, 3, 3, t);
		cvInvert(R, invR);
		cvMatMul(invR, t, t);
		fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
		fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
		fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
		fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;
	}	

	fout.close();

	cvReleaseMat(&R);
	cvReleaseMat(&t);
	cvReleaseMat(&invK);
	cvReleaseMat(&temp33);
	cvReleaseMat(&temp34);
	cvReleaseMat(&invR);
	cvReleaseMat(&P);
}

void SaveCameraData(string filename, vector<CvMat*> cP, CvMat *K, vector<int> vUsedFrame, int nFrames, int nCam)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumFrames " << nFrames << endl;
	fout << "NumCams " << nCam << endl;
	fout << "NumP " << cP.size() << endl;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *t = cvCreateMat(3,1,CV_32FC1);
	CvMat *invK = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K, invK);
	CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
	CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
	CvMat *invR = cvCreateMat(3,3,CV_32FC1);
	CvMat *P = cvCreateMat(3,4,CV_32FC1);
	for (int i = 0; i < cP.size(); i++)
	{
		P = cvCloneMat(cP[i]);
		cvMatMul(invK, P, temp34);
		GetSubMatColwise(temp34, 0, 2, R);
		GetSubMatColwise(temp34, 3, 3, t);
		cvInvert(R, invR);
		cvMatMul(invR, t, t);
		int frame = vUsedFrame[i]%nFrames;
		int cam = (int) vUsedFrame[i]/nFrames;
		fout << cam << " " << frame << endl;
		fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
		fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
		fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
		fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;
	}	

	fout.close();

	cvReleaseMat(&R);
	cvReleaseMat(&t);
	cvReleaseMat(&invK);
	cvReleaseMat(&temp33);
	cvReleaseMat(&temp34);
	cvReleaseMat(&invR);
	cvReleaseMat(&P);
}

void SaveCameraData(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams " << vCamera.size() << endl;
	fout << "NumFrames ";
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		fout << vCamera[iCamera].nFrames << " ";
	}	
	fout << endl;
	fout << "NumP " << cP.size() << endl;

	for (int i = 0; i < cP.size(); i++)
	{
		int frame = vUsedFrame[i]%max_nFrames;
		int cam = (int) vUsedFrame[i]/max_nFrames;

		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *invK = cvCreateMat(3,3,CV_32FC1);
		cvInvert(vCamera[cam].K, invK);
		CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
		CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
		CvMat *invR = cvCreateMat(3,3,CV_32FC1);
		CvMat *P = cvCreateMat(3,4,CV_32FC1);

		P = cvCloneMat(cP[i]);
		cvMatMul(invK, P, temp34);
		GetSubMatColwise(temp34, 0, 2, R);
		GetSubMatColwise(temp34, 3, 3, t);
		cvInvert(R, invR);
		cvMatMul(invR, t, t);

		fout << cam << " " << frame << endl;
		fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
		fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
		fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
		fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;

		cvReleaseMat(&R);
		cvReleaseMat(&t);
		cvReleaseMat(&invK);
		cvReleaseMat(&temp33);
		cvReleaseMat(&temp34);
		cvReleaseMat(&invR);
		cvReleaseMat(&P);
	}	

	fout.close();
}

void SaveCameraData_KRT(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams " << vCamera.size() << endl;
	fout << "NumFrames ";
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		fout << vCamera[iCamera].nFrames << " ";
	}	
	fout << endl;
	fout << "NumP " << cP.size() << endl;

	for (int i = 0; i < cP.size(); i++)
	{
		int frame = vUsedFrame[i]%max_nFrames;
		int cam = (int) vUsedFrame[i]/max_nFrames;
		vector<int>:: const_iterator it = find(vCamera[cam].vTakenFrame.begin(), vCamera[cam].vTakenFrame.end(), frame);
		if (it == vCamera[cam].vTakenFrame.end())
			return;
		int iTakenFrame = (int) (it - vCamera[cam].vTakenFrame.begin());

		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *invK = cvCreateMat(3,3,CV_32FC1);
		cvInvert(vCamera[cam].vK[iTakenFrame], invK);
		CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
		CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
		CvMat *invR = cvCreateMat(3,3,CV_32FC1);
		CvMat *P = cvCreateMat(3,4,CV_32FC1);

		P = cvCloneMat(cP[i]);
		cvMatMul(invK, P, temp34);
		GetSubMatColwise(temp34, 0, 2, R);
		GetSubMatColwise(temp34, 3, 3, t);
		cvInvert(R, invR);
		cvMatMul(invR, t, t);

		fout << vCamera[cam].id << " " << iTakenFrame << endl;
		fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
		fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
		fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
		fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;

		cvReleaseMat(&R);
		cvReleaseMat(&t);
		cvReleaseMat(&invK);
		cvReleaseMat(&temp33);
		cvReleaseMat(&temp34);
		cvReleaseMat(&invR);
		cvReleaseMat(&P);
	}	

	fout.close();
}

void SaveCameraData_KRT_TEMPORAL(string filename, vector<Camera> vCamera, int max_nFrames)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams " << vCamera.size() << endl;
	fout << "NumFrames ";
	int numP = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		fout << vCamera[iCamera].nFrames << " ";
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
			numP++;
	}	
	fout << endl;
	fout << "NumP " << numP << endl;

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			CvMat *R = cvCreateMat(3,3,CV_32FC1);
			CvMat *t = cvCreateMat(3,1,CV_32FC1);
			CvMat *invK = cvCreateMat(3,3,CV_32FC1);
			cvInvert(vCamera[iCamera].vK[iFrame], invK);
			CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
			CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
			CvMat *invR = cvCreateMat(3,3,CV_32FC1);
			CvMat *P = cvCreateMat(3,4,CV_32FC1);

			P = cvCloneMat(vCamera[iCamera].vP[iFrame]);
			cvMatMul(invK, P, temp34);
			GetSubMatColwise(temp34, 0, 2, R);
			GetSubMatColwise(temp34, 3, 3, t);
			cvInvert(R, invR);
			cvMatMul(invR, t, t);
			CvMat *q = cvCreateMat(4,1,CV_32FC1);
			Rotation2Quaternion(R, q);
			Quaternion2Rotation(q, R);

			fout << vCamera[iCamera].id << " " << iFrame << endl;
			fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
			fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
			fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
			fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;

			cvReleaseMat(&R);
			cvReleaseMat(&t);
			cvReleaseMat(&invK);
			cvReleaseMat(&temp33);
			cvReleaseMat(&temp34);
			cvReleaseMat(&invR);
			cvReleaseMat(&P);
		}
	}

	fout.close();
}

void SaveCameraData_KRT_TEMPORAL(string filename_camera, string filename_intrinsic, vector<Camera> vCamera, int max_nFrames)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename_camera << endl;
	ofstream fout;
	fout.open(filename_camera.c_str());
	fout << "NumCams " << vCamera.size() << endl;
	fout << "NumFrames ";
	int numP = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		fout << vCamera[iCamera].nFrames << " ";
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
			numP++;
	}	
	fout << endl;
	fout << "NumP " << numP << endl;

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			CvMat *R = cvCreateMat(3,3,CV_32FC1);
			CvMat *t = cvCreateMat(3,1,CV_32FC1);
			CvMat *invK = cvCreateMat(3,3,CV_32FC1);
			cvInvert(vCamera[iCamera].vK[iFrame], invK);
			CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
			CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
			CvMat *invR = cvCreateMat(3,3,CV_32FC1);
			CvMat *P = cvCreateMat(3,4,CV_32FC1);

			P = cvCloneMat(vCamera[iCamera].vP[iFrame]);
			cvMatMul(invK, P, temp34);
			GetSubMatColwise(temp34, 0, 2, R);
			GetSubMatColwise(temp34, 3, 3, t);
			cvInvert(R, invR);
			cvMatMul(invR, t, t);

			fout << vCamera[iCamera].id << " " << iFrame << " " << vCamera[iCamera].vTakenFrame[iFrame] << " " << vCamera[iCamera].vTakenInstant[iFrame] << endl;
			fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
			fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
			fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
			fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;

			cvReleaseMat(&R);
			cvReleaseMat(&t);
			cvReleaseMat(&invK);
			cvReleaseMat(&temp33);
			cvReleaseMat(&temp34);
			cvReleaseMat(&invR);
			cvReleaseMat(&P);
		}
	}

	fout.close();

	PrintAlgorithm("Save Camera Intrinsic Data");
	cout << "File name: " << filename_intrinsic << endl;
	fout.open(filename_intrinsic.c_str());
	fout << "NumCams " << vCamera.size() << endl;
	fout << "NumFrames ";
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		fout << vCamera[iCamera].nFrames << " ";
	}	
	fout << endl;
	fout << "NumP " << numP << endl;

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			fout << vCamera[iCamera].id << " " << iFrame << " " << vCamera[iCamera].vTakenFrame[iFrame] << " " << vCamera[iCamera].vTakenInstant[iFrame] << endl;
			CvMat *K = cvCreateMat(3,3,CV_32FC1);
			K = cvCloneMat(vCamera[iCamera].vK[iFrame]);
			fout << cvGetReal2D(K, 0, 0) << " " << cvGetReal2D(K, 0, 1) << " " << cvGetReal2D(K, 0, 2) << endl;
			fout << cvGetReal2D(K, 1, 0) << " " << cvGetReal2D(K, 1, 1) << " " << cvGetReal2D(K, 1, 2) << endl;
			fout << cvGetReal2D(K, 2, 0) << " " << cvGetReal2D(K, 2, 1) << " " << cvGetReal2D(K, 2, 2) << endl;
			fout << vCamera[iCamera].vk1[iFrame] << " " << vCamera[iCamera].vk2[iFrame] << endl;
			cvReleaseMat(&K);
		}
	}
	fout.close();
}

void SaveCameraData_KRT_Tempo(string filename, vector<Camera> vCamera, int max_nFrames)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams " << vCamera.size() << endl;
	fout << "NumFrames ";
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		fout << vCamera[iCamera].nFrames << " ";
	}	
	int totalFrame = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
			totalFrame++;
	}
	fout << endl;
	fout << "NumP " << totalFrame << endl;

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			CvMat *R = cvCreateMat(3,3,CV_32FC1);
			CvMat *t = cvCreateMat(3,1,CV_32FC1);
			CvMat *invK = cvCreateMat(3,3,CV_32FC1);
			cvInvert(vCamera[iCamera].vK[iFrame], invK);
			CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
			CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
			CvMat *invR = cvCreateMat(3,3,CV_32FC1);
			CvMat *P = cvCreateMat(3,4,CV_32FC1);

			P = cvCloneMat(vCamera[iCamera].vP[iFrame]);
			cvMatMul(invK, P, temp34);
			GetSubMatColwise(temp34, 0, 2, R);
			GetSubMatColwise(temp34, 3, 3, t);
			cvInvert(R, invR);
			cvMatMul(invR, t, t);

			fout << vCamera[iCamera].id << " " << vCamera[iCamera].vTakenFrame[iFrame] << endl;
			fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
			fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
			fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
			fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;

			cvReleaseMat(&R);
			cvReleaseMat(&t);
			cvReleaseMat(&invK);
			cvReleaseMat(&temp33);
			cvReleaseMat(&temp34);
			cvReleaseMat(&invR);
			cvReleaseMat(&P);
		}
	}

	fout.close();
}


void SaveCameraData_KRT_intrinsic(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams " << vCamera.size() << endl;
	fout << "NumFrames ";
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		fout << vCamera[iCamera].nFrames << " ";
	}	
	fout << endl;
	fout << "NumP " << cP.size() << endl;

	for (int i = 0; i < cP.size(); i++)
	{
		int frame = vUsedFrame[i]%max_nFrames;
		int cam = (int) vUsedFrame[i]/max_nFrames;
		vector<int>:: const_iterator it = find(vCamera[cam].vTakenFrame.begin(), vCamera[cam].vTakenFrame.end(), frame);
		if (it == vCamera[cam].vTakenFrame.end())
			return;
		int iTakenFrame = (int) (it - vCamera[cam].vTakenFrame.begin());

		fout << vCamera[cam].id << " " << iTakenFrame << endl;
		CvMat *K = cvCloneMat(vCamera[cam].vK[iTakenFrame]);
		fout << cvGetReal2D(K, 0, 0) << " " << cvGetReal2D(K, 0, 1) << " " << cvGetReal2D(K, 0, 2) << endl;
		fout << cvGetReal2D(K, 1, 0) << " " << cvGetReal2D(K, 1, 1) << " " << cvGetReal2D(K, 1, 2) << endl;
		fout << cvGetReal2D(K, 2, 0) << " " << cvGetReal2D(K, 2, 1) << " " << cvGetReal2D(K, 2, 2) << endl;

		fout << vCamera[cam].vk1[iTakenFrame] << " " << vCamera[cam].vk2[iTakenFrame] << endl;

		cvReleaseMat(&K);
	}	

	fout.close();
}





void SaveStructureData(string filename, CvMat *X)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << X->rows << endl;
	for (int i = 0; i < X->rows; i++)
	{
		fout << cvGetReal2D(X, i, 0) << " " << cvGetReal2D(X, i, 1) << " " << cvGetReal2D(X, i, 2) << endl;
	}	
	fout.close();
}

void SaveStructureData(string filename, CvMat *X, vector<int> visibleID)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << visibleID.size() << endl;
	for (int i = 0; i < visibleID.size(); i++)
	{
		fout << visibleID[i] << " " << cvGetReal2D(X, visibleID[i], 0) << " " << cvGetReal2D(X, visibleID[i], 1) << " " << cvGetReal2D(X, visibleID[i], 2) << endl;
	}	
	fout.close();
}

void SaveStructureData_RGB(string filename, CvMat *X, vector<int> visibleID, vector<Feature> vFeature)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << visibleID.size() << endl;
	for (int i = 0; i < visibleID.size(); i++)
	{
		fout << visibleID[i] << " ";
		fout << vFeature[visibleID[i]].r << " " << vFeature[visibleID[i]].g << " " << vFeature[visibleID[i]].b << " "; 
		fout << cvGetReal2D(X, visibleID[i], 0) << " " << cvGetReal2D(X, visibleID[i], 1) << " " << cvGetReal2D(X, visibleID[i], 2) << endl;
	}	
	fout.close();
}

void SaveStructureData_RGB_fast(string filename, CvMat *X, vector<Feature> &vFeature)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	int count = 0;
	for (int i = 0; i < vFeature.size(); i++)
	{
		if (vFeature[i].isRegistered)
			count++;
	}
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << count << endl;
	for (int i = 0; i < vFeature.size(); i++)
	{
		if (vFeature[i].isRegistered)
		{
			fout << vFeature[i].id << " ";
			fout << vFeature[i].r << " " << vFeature[i].g << " " << vFeature[i].b << " "; 
			fout << cvGetReal2D(X, i, 0) << " " << cvGetReal2D(X, i, 1) << " " << cvGetReal2D(X, i, 2) << endl;
		}
	}	
	fout.close();
}

void LoadStructureData(string filename, vector<int> &vID, vector<double> &vx, vector<double> &vy, vector<double> &vz)
{
	PrintAlgorithm("Load Structure Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFeatures;
	fin >> a >> nFeatures;
	for (int i = 0; i < nFeatures; i++)
	{
		int id;
		double x, y, z;
		int r,g,b;
		fin >> id >> r >> g >> b >> x >> y >> z;
		vx.push_back(x);
		vy.push_back(y);
		vz.push_back(z);
		vID.push_back(id);
	}	
	fin.close();
}

void LoadStructureData(string filename, vector<double> &vx, vector<double> &vy, vector<double> &vz)
{
	PrintAlgorithm("Load Structure Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFeatures;
	fin >> a >> nFeatures;
	for (int i = 0; i < nFeatures; i++)
	{
		int id;
		double x, y, z;
		fin >> id >> x >> y >> z;
		vx.push_back(x);
		vy.push_back(y);
		vz.push_back(z);
	}	
	fin.close();
}

void LoadFileListData(string filename, vector<string> &vFilename)
{
	PrintAlgorithm("Load FileList Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	while(!fin.eof())
	{
		string filelist;
		fin >> filelist;
		vFilename.push_back(filelist);
	}
	vFilename.pop_back();
	fin.close();
}

void SaveCorrespondence2D3DData(string filename, vector<Correspondence2D3D> vCorr, int frame, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);

	if (vCorr.size() == 0)
	{
		file.close();
		return;
	}

	file << frame << " " << vCorr.size() << " ";
	for (int iCorr = 0; iCorr < vCorr.size(); iCorr++)
	{	
		file << vCorr[iCorr].id_2D << " " << vCorr[iCorr].id_3D << " " << vCorr[iCorr].u << " " << vCorr[iCorr].v << " " << vCorr[iCorr].x << " " << vCorr[iCorr].y << " " << vCorr[iCorr].z << " ";
	}
	file << endl;
	file.close();
}

void ResaveAbsoluteCameraData(string filename, int nFrames)
{
	ifstream fin;
	fin.open(filename.c_str(), ios_base::in);
	char a[100000];
	vector<string> vS;
	while (!fin.eof())
	{
		string s;
		fin.getline(a, 100000);
		s = a;
		vS.push_back(s);
	}
	vS.pop_back();
	fin.close();

	ofstream fout;
	fout.open(filename.c_str(), ios_base::out);

	fout << "NumFrames " << nFrames << endl;;
	fout << "NumCams " << 1 << endl;
	fout << "NumP " << nFrames << endl;

	for (int i = 0; i < vS.size(); i++)
	{
		fout << vS[i] << endl;
	}
	fout.close();
}

void SaveAbsoluteCameraData(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, CvMat *K)
{
	ofstream fout;
	fout.open(filename.c_str(), ios_base::out);

	fout << "NumCams: 1" << endl;
	fout << "NumFrames: " << nTotalFrame << endl;
	fout << "NumP: " << vFrame.size() << endl;

	for (int iP = 0; iP < vP.size(); iP++)
	{
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *invK = cvCreateMat(3,3,CV_32FC1);
		cvInvert(K, invK);
		CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
		CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
		CvMat *invR = cvCreateMat(3,3,CV_32FC1);

		cvMatMul(invK, vP[iP], temp34);
		GetSubMatColwise(temp34, 0, 2, R);
		GetSubMatColwise(temp34, 3, 3, t);
		cvInvert(R, invR);
		cvMatMul(invR, t, t);

		fout << "0 " << vFrame[iP] << endl;
		fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
		fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
		fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
		fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;

		cvReleaseMat(&R);
		cvReleaseMat(&t);
		cvReleaseMat(&invK);
		cvReleaseMat(&temp33);
		cvReleaseMat(&temp34);
		cvReleaseMat(&invR);
	}
	
	fout.close();
}

void ResaveCorrespondence2D3DData(string filename, int nFiles)
{
	ifstream fin;
	fin.open(filename.c_str(), ios_base::in);
	char a[10000000];
	vector<string> vS;
	while (!fin.eof())
	{
		string s;
		fin.getline(a, 10000000);
		s = a;
		vS.push_back(s);
	}
	vS.pop_back();
	fin.close();

	ofstream fout;
	fout.open(filename.c_str(), ios_base::out);
	fout << "TotalFrames: " << nFiles << endl;
	fout << "NumReconstructedFrames: " << vS.size() << endl;
	for (int i = 0; i < vS.size(); i++)
	{
		fout << vS[i] << endl;
	}
	fout.close();
}

void LoadDescriptorData(string filename, vector<vector<double> > &vvDesc)
{
	PrintAlgorithm("Load Descpritor Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int dummy;
	while(!fin.eof())
	{
		fin >> dummy;
		fin >> dummy;
		vector<double> vDesc;
		for (int i = 0; i < 128; i++)
		{
			fin >> dummy;
			vDesc.push_back(dummy);
		}
		vvDesc.push_back(vDesc);
	}
	vvDesc.pop_back();
	fin.close();
}

void LoadDescriptorAllData_MaxDistance(string filename, vector<double> &vMaxDistance)
{
	PrintAlgorithm("Load Descpritor Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int dummy;
	int k = 0;
	while(!fin.eof())
	{
		if (k%1000 == 0)
		{
			cout << k << endl;
		}
		k++;
		fin >> dummy;
		int nDesc;
		fin >> nDesc;
		vector<vector<double> > vvDesc;
		for (int iDesc = 0; iDesc < nDesc; iDesc++)
		{
			vector<double> vDesc;
			for (int i = 0; i < 128; i++)
			{
				fin >> dummy;
				vDesc.push_back(dummy);
			}
			vvDesc.push_back(vDesc);
		}
		vector<double> AveDesc;
		AveDesc.resize(128,0);
		for (int iDesc = 0; iDesc < vvDesc.size(); iDesc++)
		{
			for (int i = 0; i < 128; i++)
			{
				AveDesc[i] = AveDesc[i] + vvDesc[iDesc][i];
			}
		}

		vector<double> vDist;
		for (int iDesc = 0; iDesc < vvDesc.size(); iDesc++)
		{
			double dist = 0;
			for (int i = 0; i < 128; i++)
			{
				dist += (AveDesc[i]/vvDesc.size() - vvDesc[iDesc][i])*(AveDesc[i]/vvDesc.size() - vvDesc[iDesc][i]);
			}
			vDist.push_back(sqrt(dist));
		}
		
		double max_dist = -1;
		for (int iDist = 0; iDist < vDist.size(); iDist++)
		{
			if (max_dist < vDist[iDist])
				max_dist = vDist[iDist];
		}
		vMaxDistance.push_back(max_dist);
	}
	fin.close();
}

void LoadDescriptorAllData_MaxDistance_Element(string filename, vector<double> &vMaxDistance, vector<vector<double> > &vvMaxDistanceElement)
{
	PrintAlgorithm("Load Descpritor Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int dummy;
	int k = 0;
	while(!fin.eof())
	{
		if (k%1000 == 0)
		{
			cout << k << endl;
		}
		k++;
		fin >> dummy;
		int nDesc;
		fin >> nDesc;
		vector<vector<double> > vvDesc;
		for (int iDesc = 0; iDesc < nDesc; iDesc++)
		{
			vector<double> vDesc;
			for (int i = 0; i < 128; i++)
			{
				fin >> dummy;
				vDesc.push_back(dummy);
			}
			vvDesc.push_back(vDesc);
		}
		vector<double> AveDesc;
		AveDesc.resize(128,0);
		for (int iDesc = 0; iDesc < vvDesc.size(); iDesc++)
		{
			for (int i = 0; i < 128; i++)
			{
				AveDesc[i] = AveDesc[i] + vvDesc[iDesc][i];
			}
		}

		vector<double> vDist;
		for (int iDesc = 0; iDesc < vvDesc.size(); iDesc++)
		{
			double dist = 0;
			for (int i = 0; i < 128; i++)
			{
				dist += (AveDesc[i]/vvDesc.size() - vvDesc[iDesc][i])*(AveDesc[i]/vvDesc.size() - vvDesc[iDesc][i]);
			}
			vDist.push_back(sqrt(dist));
		}
		
		double max_dist = -1;
		for (int iDist = 0; iDist < vDist.size(); iDist++)
		{
			if (max_dist < vDist[iDist])
				max_dist = vDist[iDist];
		}
		vMaxDistance.push_back(max_dist);

		vector<double> vMaxDistance_element;
		for (int iDim = 0; iDim < 128; iDim++)
		{
			double dist = 0;
			for (int iDesc = 0; iDesc < vvDesc.size(); iDesc++)
			{
				dist += (AveDesc[iDim]/vvDesc.size()-vvDesc[iDesc][iDim])*(AveDesc[iDim]/vvDesc.size()-vvDesc[iDesc][iDim]);
			}
			vMaxDistance_element.push_back(dist/vvDesc.size());
		}
		vvMaxDistanceElement.push_back(vMaxDistance_element);
	}
	fin.close();
}

void SaveMaxDistanceElementData(string filename, vector<double> vMaxDistance, vector<vector<double> > vvMaxDistanceElement)
{
	PrintAlgorithm("Save Vector Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << vMaxDistance.size() << endl;
	for (int i = 0; i < vMaxDistance.size(); i++)
	{
		fout << vMaxDistance[i] << " ";
		for (int j = 0; j < 128; j++)
		{
			fout << vvMaxDistanceElement[i][j] << " ";
		}
		fout << endl;
	}	
	fout.close();
}

void SaveVectorData(string filename, vector<double> vVector)
{
	PrintAlgorithm("Save Vector Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << vVector.size() << endl;
	for (int i = 0; i < vVector.size(); i++)
	{
		fout << vVector[i] << endl;
	}	
	fout.close();
}

void LoadStructureData(string filename, vector<int> &visibleID, vector<double> &vx, vector<double> &vy, vector<double> &vz, vector<int> &vr,vector<int> &vg,vector<int> &vb)
{
	PrintAlgorithm("Load Structure Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFeatures;
	fin >> a >> nFeatures;
	for (int i = 0; i < nFeatures; i++)
	{
		int id;
		double x, y, z;
		int r, g, b;
		fin >> id >> r >> g >> b >> x >> y >> z;
		visibleID.push_back(id);
		vx.push_back(x);
		vy.push_back(y);
		vz.push_back(z);
		vr.push_back(r);
		vg.push_back(g);
		vb.push_back(b);
	}	
	fin.close();
}

void LoadStructureData(string filename, vector<StaticStructure> &vStaticStructure)
{
	PrintAlgorithm("Load Structure Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFeatures;
	fin >> a >> nFeatures;
	for (int i = 0; i < nFeatures; i++)
	{
		int id;
		double x, y, z;
		int r, g, b;
		fin >> id >> r >> g >> b >> x >> y >> z;
		StaticStructure ss;
		ss.id = id;
		ss.x = x;
		ss.y = y;
		ss.z = z;
		ss.r = r;
		ss.g = g;
		ss.b = b;
		vStaticStructure.push_back(ss);
	}	
	fin.close();
}

void SaveStructureData(string filename, vector<int> visibleID, vector<double> vx, vector<double> vy, vector<double> vz, vector<int> vr,vector<int> vg,vector<int> vb)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << visibleID.size() << endl;
	for (int i = 0; i < visibleID.size(); i++)
	{
		fout << visibleID[i] << " ";
		fout << vr[i] << " " << vg[i] << " " << vb[i] << " "; 
		fout << vx[i] << " " << vy[i] << " " << vz[i] << endl;
	}	
	fout.close();
}
void LoadStructureData_Add(string filename, int nFeatures, CvMat &X, vector<int> &vVisibleID)
{
	PrintAlgorithm("Load Structure Data");
	cout << "File name: " << filename << endl;
	X = *cvCreateMat(nFeatures, 3, CV_32FC1);
	cvSetZero(&X);
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFeatures_;
	fin >> a >> nFeatures_;
	vVisibleID.clear();
	for (int i = 0; i < nFeatures_; i++)
	{
		int id;
		double x, y, z;
		fin >> id >> x >> y >> z;
		cvSetReal2D(&X, id, 0, x);
		cvSetReal2D(&X, id, 1, y);
		cvSetReal2D(&X, id, 2, z);
		vVisibleID.push_back(id);
	}	
	fin.close();
}

void LoadCalibrationData(string filename, CvMat *K, double &k1, double &k2)
{
	PrintAlgorithm("Load Calibration Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a;
	double k11, k12, k13, k21, k22, k23, k31, k32, k33;
	file >> k11 >> k12 >> k13;
	file >> k21 >> k22 >> k23;
	file >> k31 >> k32 >> k33;
	file >> k1 >> k2;

	//K = *cvCreateMat(3,3,CV_32FC1);
	cvSetReal2D(K, 0, 0, k11);		cvSetReal2D(K, 0, 1, k12);		cvSetReal2D(K, 0, 2, k13);
	cvSetReal2D(K, 1, 0, k21);		cvSetReal2D(K, 1, 1, k22);		cvSetReal2D(K, 1, 2, k23);
	cvSetReal2D(K, 2, 0, k31);		cvSetReal2D(K, 2, 1, k32);		cvSetReal2D(K, 2, 2, k33);

	file.close();
}

void LoadCalibrationData(vector<string> vFilename, vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Calibration Data");
	
	for (int iFile = 0; iFile < vFilename.size(); iFile++)
	{
		cout << "File name: " << vFilename[iFile] << endl;
		ifstream file;
		string a;
		int nCameras;
		file.open(vFilename[iFile].c_str(), ifstream::in);
		file >> a;
		CvMat *K;
		double k1, k2;
		double k11, k12, k13, k21, k22, k23, k31, k32, k33;
		file >> k11 >> k12 >> k13;
		file >> k21 >> k22 >> k23;
		file >> k31 >> k32 >> k33;
		file >> k1 >> k2;

		K = cvCreateMat(3,3,CV_32FC1);
		cvSetReal2D(K, 0, 0, k11);		cvSetReal2D(K, 0, 1, k12);		cvSetReal2D(K, 0, 2, k13);
		cvSetReal2D(K, 1, 0, k21);		cvSetReal2D(K, 1, 1, k22);		cvSetReal2D(K, 1, 2, k23);
		cvSetReal2D(K, 2, 0, k31);		cvSetReal2D(K, 2, 1, k32);		cvSetReal2D(K, 2, 2, k33);
		vCamera[iFile].K = K;
		vCamera[iFile].k1 = k1;
		vCamera[iFile].k2 = k2;
		file.close();
	}
}


void LoadCalibrationData(string filename, CvMat &K)
{
	PrintAlgorithm("Load Calibration Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a;
	double k11, k12, k13, k21, k22, k23, k31, k32, k33;
	file >> k11 >> k12 >> k13;
	file >> k21 >> k22 >> k23;
	file >> k31 >> k32 >> k33;

	K = *cvCreateMat(3,3,CV_32FC1);
	cvSetReal2D(&K, 0, 0, k11);		cvSetReal2D(&K, 0, 1, k12);		cvSetReal2D(&K, 0, 2, k13);
	cvSetReal2D(&K, 1, 0, k21);		cvSetReal2D(&K, 1, 1, k22);		cvSetReal2D(&K, 1, 2, k23);
	cvSetReal2D(&K, 2, 0, k31);		cvSetReal2D(&K, 2, 1, k32);		cvSetReal2D(&K, 2, 2, k33);

	file.close();
}

void LoadCalibrationData_INAUGURATION(string filename, Camera &cam)
{
	PrintAlgorithm("Load Calibration Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	while (!file.eof())
	{
		int frame, resolutionx, resolutiony;
		double focal_length, ccdx, ccdy;
		file >> frame >> focal_length >> ccdx >> ccdy >> resolutionx >> resolutiony;
		cam.vTakenFrame.push_back(frame);
		CvMat *K = cvCreateMat(3,3,CV_32FC1);
		cvSetIdentity(K);
		if (focal_length != 0)
		{
			cvSetReal2D(K, 0, 0, focal_length*resolutionx/ccdx);
			cvSetReal2D(K, 1, 1, focal_length*resolutiony/ccdy);
			cvSetReal2D(K, 0, 2, resolutionx/2);
			cvSetReal2D(K, 1, 2, resolutiony/2);
		}

		cam.vK.push_back(K);
		cam.vk1.push_back(0);
		cam.vk2.push_back(0);
	}
	cam.vTakenFrame.pop_back();
	cam.vK.pop_back();
	cam.vk1.pop_back();
	cam.vk2.pop_back();
	file.close();
}

void LoadCalibrationData_INAUGURATION(string filename, vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Calibration Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	while (!file.eof())
	{
		int camera, frame, resolutionx, resolutiony;
		double focal_length, ccdx, ccdy;
		file >> camera >> frame >> focal_length >> ccdx >> ccdy >> resolutionx >> resolutiony;
		int idx = -1;
		bool isFound = false;
		for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
		{
			if (vCamera[iCamera].id == camera)
			{
				idx = iCamera;
				isFound = true;
				break;
			}
		}
		if (isFound)
		{
			vCamera[idx].vTakenFrame.push_back(frame);
			CvMat *K = cvCreateMat(3,3,CV_32FC1);
			cvSetIdentity(K);
			if (focal_length != 0)
			{
				cvSetReal2D(K, 0, 0, focal_length*resolutionx/ccdx);
				cvSetReal2D(K, 1, 1, focal_length*resolutionx/ccdx);
				cvSetReal2D(K, 0, 2, resolutionx/2);
				cvSetReal2D(K, 1, 2, resolutiony/2);
			}

			vCamera[idx].vK.push_back(K);
			vCamera[idx].vk1.push_back(0);
			vCamera[idx].vk2.push_back(0);
		}
		else
		{
			Camera cam;
			cam.id = camera;
			cam.vTakenFrame.push_back(frame);
			CvMat *K = cvCreateMat(3,3,CV_32FC1);
			cvSetIdentity(K);
			if (focal_length != 0)
			{
				cvSetReal2D(K, 0, 0, focal_length*resolutionx/ccdx);
				cvSetReal2D(K, 1, 1, focal_length*resolutionx/ccdx);
				cvSetReal2D(K, 0, 2, resolutionx/2);
				cvSetReal2D(K, 1, 2, resolutiony/2);
			}

			cam.vK.push_back(K);
			cam.vk1.push_back(0);
			cam.vk2.push_back(0);
			vCamera.push_back(cam);
		}
	}
	vCamera[vCamera.size()-1].vTakenFrame.pop_back();
	vCamera[vCamera.size()-1].vk1.pop_back();
	vCamera[vCamera.size()-1].vk2.pop_back();
	vCamera[vCamera.size()-1].vK.pop_back();
	file.close();
}

string FilePathGeneration(FileName fn, int number)
{
	string sNumber;
	char cFileNum[10];
	//itoa(number, cFileNum, 10);
	string fileNum;
	fileNum.assign(cFileNum);

	for (int i = 0; i < fn.nDigit; i++)
	{
		sNumber += "0";
	}
	size_t sz = fileNum.size();
	int k = 0;
	for (int i = fn.nDigit-sz; i < fn.nDigit; i++)
		sNumber[i] = fileNum[k++];

	return fn.path + fn.prefix + sNumber + fn.suffix + "." + fn.extension;
}

void SaveMeasurementData(string filename, vector<Feature> vFeature, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		//if (vFeature[iFeature].vFrame.size() < 4)
		//	continue;
		//if (vFeature[iFeature].isInlier[0] == FEATURE_PROPERTY_DYNAMIC)
		//	file << "D ";
		//else
		//	file << "S ";
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << " " << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i];
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_RGB(string filename, vector<Feature> vFeature, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		file << vFeature[iFeature].r << " "<< vFeature[iFeature].g << " "<< vFeature[iFeature].b << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_RGB_DESC(string filename, vector<Feature> vFeature, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		file << vFeature[iFeature].r << " "<< vFeature[iFeature].g << " "<< vFeature[iFeature].b << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
			file << vFeature[iFeature].vx_dis[i] << " " << vFeature[iFeature].vy_dis[i] << " ";
			for (int id = 0; id < 128; id++)
			{
				file << vFeature[iFeature].vvDesc[i][id] << " ";
			}
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_RGB_DESC_ScaleDirection(string filename, vector<Feature> vFeature, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		file << vFeature[iFeature].r << " "<< vFeature[iFeature].g << " "<< vFeature[iFeature].b << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
			file << vFeature[iFeature].vx_dis[i] << " " << vFeature[iFeature].vy_dis[i] << " ";
			file << vFeature[iFeature].vScale[i] << " " << vFeature[iFeature].vDirection[i] << " ";
			for (int id = 0; id < 128; id++)
			{
				file << vFeature[iFeature].vvDesc[i][id] << " ";
			}
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_RGB_NODESC(string filename, vector<Feature> &vFeature, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		file << vFeature[iFeature].r << " "<< vFeature[iFeature].g << " "<< vFeature[iFeature].b << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
			file << vFeature[iFeature].vx_dis[i] << " " << vFeature[iFeature].vy_dis[i] << " ";
			//for (int id = 0; id < 128; id++)
			//{
			//	file << vFeature[iFeature].vvDesc[i][id] << " ";
			//}
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_RGB_DESC1(string filename, vector<Feature> vFeature, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		file << vFeature[iFeature].r << " "<< vFeature[iFeature].g << " "<< vFeature[iFeature].b << " ";
		for (int id = 0; id < 128; id++)
		{
			file << vFeature[iFeature].vvDesc[0][id] << " ";
		}
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
			file << vFeature[iFeature].vx_dis[i] << " " << vFeature[iFeature].vy_dis[i] << " ";
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_RGB_NODESC1(string filename, vector<Feature> vFeature, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		file << vFeature[iFeature].r << " "<< vFeature[iFeature].g << " "<< vFeature[iFeature].b << " ";
		//for (int id = 0; id < 128; id++)
		//{
		//	file << vFeature[iFeature].vvDesc[0][id] << " ";
		//}
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
			file << vFeature[iFeature].vx_dis[i] << " " << vFeature[iFeature].vy_dis[i] << " ";
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_RGB_Interpolation(string filename, vector<Feature> vFeature, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		file << vFeature[iFeature].r << " "<< vFeature[iFeature].g << " "<< vFeature[iFeature].b << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
			file << vFeature[iFeature].vx_dis[i] << " " << vFeature[iFeature].vy_dis[i] << " ";
		}
		file << endl;
	}
	file.close();
}


void SaveMeasurementData(string filename, vector<Feature> vFeature, int mode, vector<int> visibleID)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < visibleID.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << visibleID[iFeature] << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData(string filename, vector<Feature> vFeature, int mode, double dynamicThreshold)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vFrame[i] << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " 0 ";
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData(string filename, vector<Feature> vFeature, int nFrames, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << iFeature << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << (int) (vFeature[iFeature].vFrame[i]-vFeature[iFeature].vCamera[i]*nFrames) << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_RGB(string filename, vector<Feature> vFeature, int nFrames, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << iFeature << " ";
		file << vFeature[iFeature].r << " " << vFeature[iFeature].g << " " << vFeature[iFeature].b << " ";
		for (int i = 0; i < vFeature[iFeature].vFrame.size(); i++)
		{
			file << vFeature[iFeature].vCamera[i] << " " << (int) (vFeature[iFeature].vFrame[i]-vFeature[iFeature].vCamera[i]*nFrames) << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " " << vFeature[iFeature].vx_dis[i] << " " << vFeature[iFeature].vy_dis[i] << " ";
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_DESC(string filename, vector<Feature> vFeature, int nFrames, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << iFeature << " " << vFeature[iFeature].vvDesc.size() << " ";
		for (int id = 0; id < vFeature[iFeature].vvDesc.size(); id++)
		{
			for (int i = 0; i < 128; i++)
			{
				file << (int) vFeature[iFeature].vvDesc[id][i] << " ";
			}
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_DESC_SURF(string filename, vector<Feature> vFeature, int nFrames, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << iFeature << " " << vFeature[iFeature].vvDesc.size() << " ";
		for (int id = 0; id < vFeature[iFeature].vvDesc.size(); id++)
		{
			for (int i = 0; i < vFeature[iFeature].vvDesc[id].size()-1; i++)
			{
				file << vFeature[iFeature].vvDesc[id][i] << " ";
			}
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_DESC_AllDescriptor(string filename, vector<Feature> vFeature, int nFrames, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << iFeature << " " << vFeature[iFeature].vvDesc_AllDescriptors.size() << " ";
		for (int id = 0; id < vFeature[iFeature].vvDesc_AllDescriptors.size(); id++)
		{
			for (int i = 0; i < 128; i++)
			{
				file << vFeature[iFeature].vvDesc_AllDescriptors[id][i] << " ";
			}
		}
		file << endl;
	}
	file.close();
}

void SaveMeasurementData_DESC_AllDescriptor_ScaleDirection(string filename, vector<Feature> vFeature, int nFrames, int mode)
{
	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
		file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << iFeature << " " << vFeature[iFeature].vvDesc_AllDescriptors.size() << " ";
		for (int id = 0; id < vFeature[iFeature].vvDesc_AllDescriptors.size(); id++)
		{
			file << vFeature[iFeature].vScale[id] << " " << vFeature[iFeature].vDirection[id] << " ";
			for (int i = 0; i < 128; i++)
			{
				file << vFeature[iFeature].vvDesc_AllDescriptors[id][i] << " ";
			}
		}
		file << endl;
	}
	file.close();
}

void ResaveMeasurementData(string filename, int nFrames, int nCameras)
{
	ifstream fin;
	fin.open(filename.c_str(), ios_base::in);
	char a[9000];
	vector<string> vS;
	while (!fin.eof())
	{
		string s;
		fin.getline(a, 9000);
		s = a;
		vS.push_back(s);
	}
	vS.pop_back();
	fin.close();

	ofstream fout;
	fout.open(filename.c_str(), ios_base::out);
	fout << "NumImages: " << nFrames << endl;
	fout << "NumCamers: " << nCameras << endl;
	fout << "Num3DPoints: " << vS.size() << endl;
	for (int i = 0; i < vS.size(); i++)
	{
		fout << vS[i] << endl;
	}
	fout.close();
}

void ResaveMeasurementData(string filename, vector<int> vnFrames, int nCameras)
{
	ifstream fin;
	fin.open(filename.c_str(), ios_base::in);
	char a[10000];
	vector<string> vS;
	while (!fin.eof())
	{
		string s;
		fin.getline(a, 10000);
		s = a;
		vS.push_back(s);
	}
	vS.pop_back();
	fin.close();

	ofstream fout;
	fout.open(filename.c_str(), ios_base::out);
	fout << "NumCameras: " << nCameras << endl;
	fout << "NumImages: ";
	for (int i = 0; i < vnFrames.size(); i++)
		fout << vnFrames[i] << " ";
	fout << endl;
	fout << "Num3DPoints: " << vS.size() << endl;
	for (int i = 0; i < vS.size(); i++)
	{
		fout << vS[i] << endl;
	}
	fout.close();
}

void ResaveMeasurementData(string filename, vector<int> vnFrames, int nCameras, int nPoints)
{
	ofstream fout;
	fout.open(filename.c_str(), ios_base::ate);
	//long a = fout.tellp();
	//fout.seekp(-100);
	fout << "NumCameras: " << nCameras << endl;
	fout << "NumImages: ";
	for (int i = 0; i < vnFrames.size(); i++)
		fout << vnFrames[i] << " ";
	fout << endl;
	fout << "Num3DPoints: " << nPoints << endl;

	fout.close();
}


void SaveThetaData(string filename, vector<Theta> vTheta, int nFeatures, int nFrames, int nCameras, int nBase)
{
	PrintAlgorithm("Save Theta Data");
	cout << "File name: " << filename << endl;
	ofstream file;
	string a;
	file.open(filename.c_str());
	file << "nFrames: " << nFrames << endl;
	file << "nCameras: " << nCameras << endl;
	file << "nFeatures: " << nFeatures << endl;
	file << "nBase: " << nBase << endl;
	
	for (int iTheta = 0; iTheta < vTheta.size(); iTheta++)
	{
		if (vTheta[iTheta].isStatic)
			file << "S ";
		else
			file << "D ";
		file << vTheta[iTheta].id << " ";
		file << vTheta[iTheta].r << " "<< vTheta[iTheta].g << " "<< vTheta[iTheta].b << " ";
		for (int iBase = 0; iBase < nBase; iBase++)
		{
			file << vTheta[iTheta].thetaX[iBase] << " ";
		}
		for (int iBase = 0; iBase < nBase; iBase++)
		{
			file << vTheta[iTheta].thetaY[iBase] << " ";
		}
		for (int iBase = 0; iBase < nBase; iBase++)
		{
			file << vTheta[iTheta].thetaZ[iBase] << " ";
		}
		file  << endl;
	}
	file.close();
}

void SaveCameraData(string filename, vector<Camera> vCamera, CvMat *K, int nFrames, int nCameras)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	int nP = 0;
	for (int i = 0; i < vCamera.size(); i++)
	{
		nP += vCamera[i].vTakenFrame.size();
	}
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumFrames " << nFrames << endl;
	fout << "NumCams " << vCamera.size() << endl;
	fout << "NumP " << nP <<endl;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *t = cvCreateMat(3,1,CV_32FC1);
	CvMat *invK = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K, invK);
	CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
	CvMat *temp33 = cvCreateMat(3,3,CV_32FC1);
	CvMat *invR = cvCreateMat(3,3,CV_32FC1);
	CvMat *P = cvCreateMat(3,4,CV_32FC1);
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			int cFrame = vCamera[iCamera].vTakenFrame[iFrame];
			int cCamera = vCamera[iCamera].id;
			P = cvCloneMat(vCamera[iCamera].vP[iFrame]);
			cvMatMul(invK, P, temp34);
			GetSubMatColwise(temp34, 0, 2, R);
			GetSubMatColwise(temp34, 3, 3, t);
			cvInvert(R, invR);
			cvMatMul(invR, t, t);
			fout << cCamera << " " << cFrame << endl;
			fout << -cvGetReal2D(t, 0, 0) << " " << -cvGetReal2D(t, 1, 0) << " " << -cvGetReal2D(t, 2, 0) << endl;
			fout << cvGetReal2D(R, 0, 0) << " " << cvGetReal2D(R, 0, 1) << " " << cvGetReal2D(R, 0, 2) << endl;
			fout << cvGetReal2D(R, 1, 0) << " " << cvGetReal2D(R, 1, 1) << " " << cvGetReal2D(R, 1, 2) << endl;
			fout << cvGetReal2D(R, 2, 0) << " " << cvGetReal2D(R, 2, 1) << " " << cvGetReal2D(R, 2, 2) << endl;
		}
	}
	fout.close();

	cvReleaseMat(&R);
	cvReleaseMat(&t);
	cvReleaseMat(&invK);
	cvReleaseMat(&temp33);
	cvReleaseMat(&temp34);
	cvReleaseMat(&invR);
	cvReleaseMat(&P);
}

void LoadDynamicObjectWindow(string filename, string path, vector<DynamicObjectWindow> &vDW)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ios_base::in);
	vDW.clear();
	string a;
	fin >> a;
	while(!fin.eof())
	{
		DynamicObjectWindow dw;
		string im1, im2;
		fin >> im1 >> dw.x1 >> dw.y1 >> dw.x2 >> dw.y2 >> im2;
		dw.originalImage = path + im1;
		dw.croppedImage = path + im2;
		vDW.push_back(dw);
	}
	vDW.pop_back();
	fin.close();
}

void LoadDynamicObjectWindow(vector<string> vFilename, vector<string> vPath, vector<DynamicObjectWindow> &vDW)
{
	PrintAlgorithm("Load Camera Data");
	vDW.clear();
	for (int iFile = 0; iFile < vFilename.size(); iFile++)
	{
		string file = vPath[iFile]+vFilename[iFile];
		cout << "File name: " << vPath[iFile]+vFilename[iFile] << endl;
		ifstream fin;
		fin.open(file.c_str(), ios_base::in);
		
		string a;
		fin >> a;
		while(!fin.eof())
		{
			DynamicObjectWindow dw;
			string im1, im2;
			fin >> im1 >> dw.x1 >> dw.y1 >> dw.x2 >> dw.y2 >> im2;
			dw.originalImage = vPath[iFile] + im1;
			dw.croppedImage = vPath[iFile] + im2;
			vDW.push_back(dw);
		}
		vDW.pop_back();
		fin.close();
	}
}

void LoadExifData(vector<string> vFilename, vector<int> vTimeoffset, vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Exif Data");
	vCamera.clear();
	for (int iFile = 0; iFile < vFilename.size(); iFile++)
	{
		cout << "File name: " << vFilename[iFile] << endl;
		ifstream fin;
		fin.open(vFilename[iFile].c_str(), ios_base::in);
		int iFrame = 0;
		Camera camera;
		while (!fin.eof())
		{
			string a;
			int time;
			fin >> a;
			fin >> time;
			time = time - vTimeoffset[iFile];
			camera.vTakenFrame.push_back(time);
		}
		camera.vTakenFrame.pop_back();
		vCamera.push_back(camera);
		fin.close();
	}
	int min_time = 1e+10;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		if (vCamera[iCamera].vTakenFrame[0] < min_time)
			min_time = vCamera[iCamera].vTakenFrame[0];
	}
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			vCamera[iCamera].vTakenFrame[iFrame] -= min_time;
		}
	}
}

void CameraDefinition(string fileinfo, string filepath, string exifFile, string calibrationFile, vector<Camera> &vCamera)
{
	LoadFileInfo(fileinfo, filepath, vCamera);
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Camera definition
	// canon
	Camera canon;
	canon.exifFileName = filepath + "canon/" + exifFile;
	canon.calibrationFilename = filepath + "canon/" + calibrationFile;
	int hour = 8; int minute = 4; int sec = 22;
	canon.timeOffset = hour*3600+minute*60+sec;
	canon.ccd_width = 22.7;
	canon.ccd_height = 15.1;


	Camera pentax;
	pentax.exifFileName = filepath + "pentax/" + exifFile;
	pentax.calibrationFilename = filepath + "pentax/" + calibrationFile;
	hour = 6; minute = 43; sec = 8;
	pentax.timeOffset = hour*3600+minute*60+sec;
	pentax.ccd_width = 6.16;
	pentax.ccd_width = 4.62;

	Camera parasonic1;
	parasonic1.exifFileName = filepath + "parasonic1/" + exifFile;
	parasonic1.calibrationFilename = filepath + "parasonic1/" + calibrationFile;
	hour = 6; minute = 40; sec = 38;
	parasonic1.timeOffset = hour*3600+minute*60+sec;
	parasonic1.ccd_width = 8.2315;
	parasonic1.ccd_height = 6.1227;

	Camera parasonic2;
	parasonic2.exifFileName = filepath + "parasonic2/" + exifFile;
	parasonic2.calibrationFilename = filepath + "parasonic2/" + calibrationFile;
	hour = 6; minute = 42; sec = 13;
	parasonic2.timeOffset = hour*3600+minute*60+sec;
	parasonic2.ccd_width = 8.2315;
	parasonic2.ccd_height = 6.1227;

	Camera parasonic3;
	parasonic3.exifFileName = filepath + "parasonic3/" + exifFile;
	parasonic3.calibrationFilename = filepath + "parasonic3/" + calibrationFile;
	hour = 6; minute = 42; sec = 48;
	parasonic3.timeOffset = hour*3600+minute*60+sec;
	parasonic3.ccd_width = 8.2315;
	parasonic3.ccd_height = 6.1227;

	Camera nikon1;
	nikon1.exifFileName = filepath + "nikon1/" + exifFile;
	nikon1.calibrationFilename = filepath + "nikon1/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon1.timeOffset = hour*3600+minute*60+sec;
	nikon1.ccd_width = 6.08;
	nikon1.ccd_height = 4.62;

	Camera nikon2;
	nikon2.exifFileName = filepath + "nikon2/" + exifFile;
	nikon2.calibrationFilename = filepath + "nikon2/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon2.timeOffset = hour*3600+minute*60+sec;
	nikon2.ccd_width = 6.08;
	nikon2.ccd_height = 4.62;

	Camera nikon3;
	nikon3.exifFileName = filepath + "nikon3/" + exifFile;
	nikon3.calibrationFilename = filepath + "nikon3/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon3.timeOffset = hour*3600+minute*60+sec;
	nikon3.ccd_width = 7.6;
	nikon3.ccd_height = 5.7;

	Camera nikon4;
	nikon4.exifFileName = filepath + "nikon4/" + exifFile;
	nikon4.calibrationFilename = filepath + "nikon4/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	//hour = 6; minute = 42; sec = 26;
	nikon4.timeOffset = hour*3600+minute*60+sec;
	nikon4.ccd_width = 7.6;
	nikon4.ccd_height = 5.7;

	Camera peter;
	peter.exifFileName = filepath + "peter/" + exifFile;
	peter.calibrationFilename = filepath + "peter/" + calibrationFile;
	hour = 6; minute = 43; sec = 8;
	peter.timeOffset = hour*3600+minute*60+sec;
	peter.ccd_width = 5.75;
	peter.ccd_height = 4.31;

	Camera yali;
	yali.exifFileName = filepath + "yali/" + exifFile;
	yali.calibrationFilename = filepath + "yali/" + calibrationFile;
	hour = 7; minute = 40; sec = 59; 
	yali.timeOffset = hour*3600+minute*60+sec;
	yali.ccd_width = 9.5;
	yali.ccd_height = 7.6;

	Camera yuko;
	yuko.exifFileName = filepath + "yuko/" + exifFile;
	yuko.calibrationFilename = filepath + "yuko/" + calibrationFile;
	hour = 6; minute = 43; sec = 10; // undefined
	yuko.timeOffset = hour*3600+minute*60+sec;
	yuko.ccd_width = 9.5;
	yuko.ccd_height = 7.6;

	Camera video1;
	video1.exifFileName = filepath + "video1/" + exifFile;
	video1.calibrationFilename = filepath + "video1/" + calibrationFile;
	hour = 6; minute = 41; sec = 57;
	video1.timeOffset = hour*3600+minute*60+sec;
	video1.ccd_width = 0;
	video1.ccd_height = 0;

	Camera video2;
	video2.exifFileName = filepath + "video2/" + exifFile;
	video2.calibrationFilename = filepath + "video2/" + calibrationFile;
	hour = 6; minute = 41; sec = 48; // undefined
	video2.timeOffset = hour*3600+minute*60+sec;
	video2.ccd_width = 0;
	video2.ccd_height = 0;

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		Camera cam;
		switch (vCamera[iCamera].id)
		{			
		case 0:
			{
				cam = canon;
				break;
			}
		case 1:
			{
				cam = pentax;
				break;
			}
		case 2:
			{
				cam = parasonic1;
				break;
			}
		case 3:
			{
				cam = parasonic2;
				break;
			}
		case 4:
			{
				cam = parasonic3;
				break;
			}
		case 5:
			{
				cam = nikon1;
				break;
			}
		case 6:
			{
				cam = nikon2;
				break;
			}
		case 7:
			{
				cam = nikon3;
				break;
			}
		case 8:
			{
				cam = nikon4;
				break;
			}
		case 10:
			{
				cam = peter;
				break;
			}
		case 11:
			{
				cam = yali;
				break;
			}
		case 12:
			{
				cam = yuko;
				break;
			}
		}
		vCamera[iCamera].exifFileName = cam.exifFileName;
		vCamera[iCamera].calibrationFilename = cam.calibrationFilename;
		vCamera[iCamera].timeOffset = cam.timeOffset;
		vCamera[iCamera].ccd_width = cam.ccd_width;
		vCamera[iCamera].ccd_height = cam.ccd_height;
	}

	int timeRef = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		if (vCamera[iCamera].timeOffset < timeRef)
			timeRef = vCamera[iCamera].timeOffset;
	}
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
		vCamera[iCamera].timeOffset -= timeRef;
}

void CameraDefinition(string fileinfo, string filepath, string exifFile, string calibrationFile, string timeoffsetFile, vector<Camera> &vCamera)
{
	LoadFileInfo(fileinfo, filepath, vCamera);
	vector<int> vTimeoffset, vCameraID;
	LoadTimeOffset(timeoffsetFile, vTimeoffset, vCameraID);
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Camera definition
	// canon
	Camera canon;
	canon.exifFileName = filepath + "canon/" + exifFile;
	canon.calibrationFilename = filepath + "canon/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 0)
		{
			canon.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	canon.ccd_width = 22.7;
	canon.ccd_height = 15.1;


	Camera pentax;
	pentax.exifFileName = filepath + "pentax/" + exifFile;
	pentax.calibrationFilename = filepath + "pentax/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 1)
		{
			pentax.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	pentax.ccd_width = 6.16;
	pentax.ccd_width = 4.62;

	Camera parasonic1;
	parasonic1.exifFileName = filepath + "parasonic1/" + exifFile;
	parasonic1.calibrationFilename = filepath + "parasonic1/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 2)
		{
			parasonic1.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	parasonic1.ccd_width = 8.2315;
	parasonic1.ccd_height = 6.1227;

	Camera parasonic2;
	parasonic2.exifFileName = filepath + "parasonic2/" + exifFile;
	parasonic2.calibrationFilename = filepath + "parasonic2/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 3)
		{
			parasonic2.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	parasonic2.ccd_width = 8.2315;
	parasonic2.ccd_height = 6.1227;

	Camera parasonic3;
	parasonic3.exifFileName = filepath + "parasonic3/" + exifFile;
	parasonic3.calibrationFilename = filepath + "parasonic3/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 4)
		{
			parasonic3.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	parasonic3.ccd_width = 8.2315;
	parasonic3.ccd_height = 6.1227;

	Camera nikon1;
	nikon1.exifFileName = filepath + "nikon1/" + exifFile;
	nikon1.calibrationFilename = filepath + "nikon1/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 5)
		{
			nikon1.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	nikon1.ccd_width = 6.08;
	nikon1.ccd_height = 4.62;

	Camera nikon2;
	nikon2.exifFileName = filepath + "nikon2/" + exifFile;
	nikon2.calibrationFilename = filepath + "nikon2/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 6)
		{
			nikon2.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	nikon2.ccd_width = 6.08;
	nikon2.ccd_height = 4.62;

	Camera nikon3;
	nikon3.exifFileName = filepath + "nikon3/" + exifFile;
	nikon3.calibrationFilename = filepath + "nikon3/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 7)
		{
			nikon3.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	nikon3.ccd_width = 7.6;
	nikon3.ccd_height = 5.7;

	Camera nikon4;
	nikon4.exifFileName = filepath + "nikon4/" + exifFile;
	nikon4.calibrationFilename = filepath + "nikon4/" + calibrationFile;
	for (int iCamID = 0; iCamID < vCameraID.size(); iCamID++)
	{
		if (vCameraID[iCamID] == 8)
		{
			nikon4.timeOffset = vTimeoffset[iCamID];
			break;
		}
	}
	nikon4.ccd_width = 7.6;
	nikon4.ccd_height = 5.7;

	Camera peter;
	peter.exifFileName = filepath + "peter/" + exifFile;
	peter.calibrationFilename = filepath + "peter/" + calibrationFile;

	peter.ccd_width = 5.75;
	peter.ccd_height = 4.31;

	Camera yali;
	yali.exifFileName = filepath + "yali/" + exifFile;
	yali.calibrationFilename = filepath + "yali/" + calibrationFile;

	yali.ccd_width = 9.5;
	yali.ccd_height = 7.6;

	Camera yuko;
	yuko.exifFileName = filepath + "yuko/" + exifFile;
	yuko.calibrationFilename = filepath + "yuko/" + calibrationFile;

	yuko.ccd_width = 9.5;
	yuko.ccd_height = 7.6;

	Camera video1;
	video1.exifFileName = filepath + "video1/" + exifFile;
	video1.calibrationFilename = filepath + "video1/" + calibrationFile;

	video1.ccd_width = 0;
	video1.ccd_height = 0;

	Camera video2;
	video2.exifFileName = filepath + "video2/" + exifFile;
	video2.calibrationFilename = filepath + "video2/" + calibrationFile;

	video2.ccd_width = 0;
	video2.ccd_height = 0;

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		Camera cam;
		switch (vCamera[iCamera].id)
		{			
		case 0:
			{
				cam = canon;
				break;
			}
		case 1:
			{
				cam = pentax;
				break;
			}
		case 2:
			{
				cam = parasonic1;
				break;
			}
		case 3:
			{
				cam = parasonic2;
				break;
			}
		case 4:
			{
				cam = parasonic3;
				break;
			}
		case 5:
			{
				cam = nikon1;
				break;
			}
		case 6:
			{
				cam = nikon2;
				break;
			}
		case 7:
			{
				cam = nikon3;
				break;
			}
		case 8:
			{
				cam = nikon4;
				break;
			}
		case 10:
			{
				cam = peter;
				break;
			}
		case 11:
			{
				cam = yali;
				break;
			}
		case 12:
			{
				cam = yuko;
				break;
			}
		}
		vCamera[iCamera].exifFileName = cam.exifFileName;
		vCamera[iCamera].calibrationFilename = cam.calibrationFilename;
		vCamera[iCamera].timeOffset = cam.timeOffset;
		vCamera[iCamera].ccd_width = cam.ccd_width;
		vCamera[iCamera].ccd_height = cam.ccd_height;
	}

	int timeRef = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		if (vCamera[iCamera].timeOffset < timeRef)
			timeRef = vCamera[iCamera].timeOffset;
	}
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
		vCamera[iCamera].timeOffset -= timeRef;
}

void CameraDefinition_HandWave(string fileinfo, string filepath, string exifFile, string calibrationFile, vector<Camera> &vCamera)
{
	LoadFileInfo(fileinfo, filepath, vCamera);
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Camera definition
	// canon
	Camera canon;
	canon.exifFileName = filepath + "canon/" + exifFile;
	canon.calibrationFilename = filepath + "canon/" + calibrationFile;
	int hour = 8; int minute = 4; int sec = 22;
	canon.timeOffset = hour*3600+minute*60+sec;
	canon.ccd_width = 22.7;
	canon.ccd_height = 15.1;


	Camera pentax;
	pentax.exifFileName = filepath + "pentax/" + exifFile;
	pentax.calibrationFilename = filepath + "pentax/" + calibrationFile;
	hour = 6; minute = 43; sec = 8;
	pentax.timeOffset = hour*3600+minute*60+sec;
	pentax.ccd_width = 6.16;
	pentax.ccd_width = 4.62;

	Camera parasonic1;
	parasonic1.exifFileName = filepath + "parasonic1/" + exifFile;
	parasonic1.calibrationFilename = filepath + "parasonic1/" + calibrationFile;
	hour = 6; minute = 40; sec = 33;
	parasonic1.timeOffset = hour*3600+minute*60+sec;
	parasonic1.ccd_width = 8.2315;
	parasonic1.ccd_height = 6.1227;

	Camera parasonic2;
	parasonic2.exifFileName = filepath + "parasonic2/" + exifFile;
	parasonic2.calibrationFilename = filepath + "parasonic2/" + calibrationFile;
	hour = 6; minute = 42; sec = 13;
	parasonic2.timeOffset = hour*3600+minute*60+sec;
	parasonic2.ccd_width = 8.2315;
	parasonic2.ccd_height = 6.1227;

	Camera parasonic3;
	parasonic3.exifFileName = filepath + "parasonic3/" + exifFile;
	parasonic3.calibrationFilename = filepath + "parasonic3/" + calibrationFile;
	hour = 6; minute = 42; sec = 45;
	parasonic3.timeOffset = hour*3600+minute*60+sec;
	parasonic3.ccd_width = 8.2315;
	parasonic3.ccd_height = 6.1227;

	Camera nikon1;
	nikon1.exifFileName = filepath + "nikon1/" + exifFile;
	nikon1.calibrationFilename = filepath + "nikon1/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon1.timeOffset = hour*3600+minute*60+sec;
	nikon1.ccd_width = 6.08;
	nikon1.ccd_height = 4.62;

	Camera nikon2;
	nikon2.exifFileName = filepath + "nikon2/" + exifFile;
	nikon2.calibrationFilename = filepath + "nikon2/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon2.timeOffset = hour*3600+minute*60+sec;
	nikon2.ccd_width = 6.08;
	nikon2.ccd_height = 4.62;

	Camera nikon3;
	nikon3.exifFileName = filepath + "nikon3/" + exifFile;
	nikon3.calibrationFilename = filepath + "nikon3/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon3.timeOffset = hour*3600+minute*60+sec;
	nikon3.ccd_width = 7.6;
	nikon3.ccd_height = 5.7;

	Camera nikon4;
	nikon4.exifFileName = filepath + "nikon4/" + exifFile;
	nikon4.calibrationFilename = filepath + "nikon4/" + calibrationFile;
	//hour = 6; minute = 43; sec = 10;
	hour = 6; minute = 42; sec = 26;
	nikon4.timeOffset = hour*3600+minute*60+sec;
	nikon4.ccd_width = 7.6;
	nikon4.ccd_height = 5.7;

	Camera peter;
	peter.exifFileName = filepath + "peter/" + exifFile;
	peter.calibrationFilename = filepath + "peter/" + calibrationFile;
	hour = 6; minute = 43; sec = 8;
	peter.timeOffset = hour*3600+minute*60+sec;
	peter.ccd_width = 5.75;
	peter.ccd_height = 4.31;

	Camera yali;
	yali.exifFileName = filepath + "yali/" + exifFile;
	yali.calibrationFilename = filepath + "yali/" + calibrationFile;
	hour = 7; minute = 40; sec = 59; 
	yali.timeOffset = hour*3600+minute*60+sec;
	yali.ccd_width = 9.5;
	yali.ccd_height = 7.6;

	Camera yuko;
	yuko.exifFileName = filepath + "yuko/" + exifFile;
	yuko.calibrationFilename = filepath + "yuko/" + calibrationFile;
	hour = 6; minute = 43; sec = 10; // undefined
	yuko.timeOffset = hour*3600+minute*60+sec;
	yuko.ccd_width = 9.5;
	yuko.ccd_height = 7.6;

	Camera video1;
	video1.exifFileName = filepath + "video1/" + exifFile;
	video1.calibrationFilename = filepath + "video1/" + calibrationFile;
	hour = 6; minute = 41; sec = 57;
	video1.timeOffset = hour*3600+minute*60+sec;
	video1.ccd_width = 0;
	video1.ccd_height = 0;

	Camera video2;
	video2.exifFileName = filepath + "video2/" + exifFile;
	video2.calibrationFilename = filepath + "video2/" + calibrationFile;
	hour = 6; minute = 41; sec = 48; // undefined
	video2.timeOffset = hour*3600+minute*60+sec;
	video2.ccd_width = 0;
	video2.ccd_height = 0;

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		Camera cam;
		switch (vCamera[iCamera].id)
		{			
		case 0:
			{
				cam = canon;
				break;
			}
		case 1:
			{
				cam = pentax;
				break;
			}
		case 2:
			{
				cam = parasonic1;
				break;
			}
		case 3:
			{
				cam = parasonic2;
				break;
			}
		case 4:
			{
				cam = parasonic3;
				break;
			}
		case 5:
			{
				cam = nikon1;
				break;
			}
		case 6:
			{
				cam = nikon2;
				break;
			}
		case 7:
			{
				cam = nikon3;
				break;
			}
		case 8:
			{
				cam = nikon4;
				break;
			}
		case 10:
			{
				cam = peter;
				break;
			}
		case 11:
			{
				cam = yali;
				break;
			}
		case 12:
			{
				cam = yuko;
				break;
			}
		}
		vCamera[iCamera].exifFileName = cam.exifFileName;
		vCamera[iCamera].calibrationFilename = cam.calibrationFilename;
		vCamera[iCamera].timeOffset = cam.timeOffset;
		vCamera[iCamera].ccd_width = cam.ccd_width;
		vCamera[iCamera].ccd_height = cam.ccd_height;
	}

	int timeRef = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		if (vCamera[iCamera].timeOffset < timeRef)
			timeRef = vCamera[iCamera].timeOffset;
	}
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
		vCamera[iCamera].timeOffset -= timeRef;
}

void CameraDefinition_FunnyMove(string fileinfo, string filepath, string exifFile, string calibrationFile, vector<Camera> &vCamera)
{
	LoadFileInfo(fileinfo, filepath, vCamera);
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Camera definition
	// canon
	Camera canon;
	canon.exifFileName = filepath + "canon/" + exifFile;
	canon.calibrationFilename = filepath + "canon/" + calibrationFile;
	int hour = 8; int minute = 4; int sec = 22;
	canon.timeOffset = hour*3600+minute*60+sec;
	canon.ccd_width = 22.7;
	canon.ccd_height = 15.1;


	Camera pentax;
	pentax.exifFileName = filepath + "pentax/" + exifFile;
	pentax.calibrationFilename = filepath + "pentax/" + calibrationFile;
	hour = 6; minute = 43; sec = 8;
	pentax.timeOffset = hour*3600+minute*60+sec;
	pentax.ccd_width = 6.16;
	pentax.ccd_width = 4.62;

	Camera parasonic1;
	parasonic1.exifFileName = filepath + "parasonic1/" + exifFile;
	parasonic1.calibrationFilename = filepath + "parasonic1/" + calibrationFile;
	hour = 6; minute = 41; sec = 18;
	parasonic1.timeOffset = hour*3600+minute*60+sec;
	parasonic1.ccd_width = 8.2315;
	parasonic1.ccd_height = 6.1227;

	Camera parasonic2;
	parasonic2.exifFileName = filepath + "parasonic2/" + exifFile;
	parasonic2.calibrationFilename = filepath + "parasonic2/" + calibrationFile;
	hour = 6; minute = 42; sec = 56;
	parasonic2.timeOffset = hour*3600+minute*60+sec;
	parasonic2.ccd_width = 8.2315;
	parasonic2.ccd_height = 6.1227;

	Camera parasonic3;
	parasonic3.exifFileName = filepath + "parasonic3/" + exifFile;
	parasonic3.calibrationFilename = filepath + "parasonic3/" + calibrationFile;
	hour = 6; minute = 43; sec = 29;
	parasonic3.timeOffset = hour*3600+minute*60+sec;
	parasonic3.ccd_width = 8.2315;
	parasonic3.ccd_height = 6.1227;

	Camera nikon1;
	nikon1.exifFileName = filepath + "nikon1/" + exifFile;
	nikon1.calibrationFilename = filepath + "nikon1/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon1.timeOffset = hour*3600+minute*60+sec;
	nikon1.ccd_width = 6.08;
	nikon1.ccd_height = 4.62;

	Camera nikon2;
	nikon2.exifFileName = filepath + "nikon2/" + exifFile;
	nikon2.calibrationFilename = filepath + "nikon2/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon2.timeOffset = hour*3600+minute*60+sec;
	nikon2.ccd_width = 6.08;
	nikon2.ccd_height = 4.62;

	Camera nikon3;
	nikon3.exifFileName = filepath + "nikon3/" + exifFile;
	nikon3.calibrationFilename = filepath + "nikon3/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon3.timeOffset = hour*3600+minute*60+sec;
	nikon3.ccd_width = 7.6;
	nikon3.ccd_height = 5.7;

	Camera nikon4;
	nikon4.exifFileName = filepath + "nikon4/" + exifFile;
	nikon4.calibrationFilename = filepath + "nikon4/" + calibrationFile;
	hour = 6; minute = 43; sec = 10;
	nikon4.timeOffset = hour*3600+minute*60+sec;
	nikon4.ccd_width = 7.6;
	nikon4.ccd_height = 5.7;

	Camera peter;
	peter.exifFileName = filepath + "peter/" + exifFile;
	peter.calibrationFilename = filepath + "peter/" + calibrationFile;
	hour = 6; minute = 43; sec = 8;
	peter.timeOffset = hour*3600+minute*60+sec;
	peter.ccd_width = 5.75;
	peter.ccd_height = 4.31;

	Camera yali;
	yali.exifFileName = filepath + "yali/" + exifFile;
	yali.calibrationFilename = filepath + "yali/" + calibrationFile;
	hour = 7; minute = 40; sec = 59; 
	yali.timeOffset = hour*3600+minute*60+sec;
	yali.ccd_width = 9.5;
	yali.ccd_height = 7.6;

	Camera yuko;
	yuko.exifFileName = filepath + "yuko/" + exifFile;
	yuko.calibrationFilename = filepath + "yuko/" + calibrationFile;
	hour = 6; minute = 43; sec = 10; // undefined
	yuko.timeOffset = hour*3600+minute*60+sec;
	yuko.ccd_width = 9.5;
	yuko.ccd_height = 7.6;

	Camera video1;
	video1.exifFileName = filepath + "video1/" + exifFile;
	video1.calibrationFilename = filepath + "video1/" + calibrationFile;
	hour = 6; minute = 41; sec = 57;
	video1.timeOffset = hour*3600+minute*60+sec;
	video1.ccd_width = 0;
	video1.ccd_height = 0;

	Camera video2;
	video2.exifFileName = filepath + "video2/" + exifFile;
	video2.calibrationFilename = filepath + "video2/" + calibrationFile;
	hour = 6; minute = 41; sec = 48; // undefined
	video2.timeOffset = hour*3600+minute*60+sec;
	video2.ccd_width = 0;
	video2.ccd_height = 0;

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		Camera cam;
		switch (vCamera[iCamera].id)
		{			
		case 0:
			{
				cam = canon;
				break;
			}
		case 1:
			{
				cam = pentax;
				break;
			}
		case 2:
			{
				cam = parasonic1;
				break;
			}
		case 3:
			{
				cam = parasonic2;
				break;
			}
		case 4:
			{
				cam = parasonic3;
				break;
			}
		case 5:
			{
				cam = nikon1;
				break;
			}
		case 6:
			{
				cam = nikon2;
				break;
			}
		case 7:
			{
				cam = nikon3;
				break;
			}
		case 8:
			{
				cam = nikon4;
				break;
			}
		case 10:
			{
				cam = peter;
				break;
			}
		case 11:
			{
				cam = yali;
				break;
			}
		case 12:
			{
				cam = yuko;
				break;
			}
		}
		vCamera[iCamera].exifFileName = cam.exifFileName;
		vCamera[iCamera].calibrationFilename = cam.calibrationFilename;
		vCamera[iCamera].timeOffset = cam.timeOffset;
		vCamera[iCamera].ccd_width = cam.ccd_width;
		vCamera[iCamera].ccd_height = cam.ccd_height;
	}

	int timeRef = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		if (vCamera[iCamera].timeOffset < timeRef)
			timeRef = vCamera[iCamera].timeOffset;
	}
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
		vCamera[iCamera].timeOffset -= timeRef;
}


void LoadExifData(vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Exif Data");
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		cout << "File name: " << vCamera[iCamera].exifFileName << endl;
		ifstream fin;
		fin.open(vCamera[iCamera].exifFileName.c_str(), ios_base::in);
		int iFrame = 0;
		vCamera[iCamera].vTakenFrame.clear();
		while (!fin.eof())
		{
			string a;
			int time;
			fin >> a;
			fin >> time;
			time = time - vCamera[iCamera].timeOffset;
			vCamera[iCamera].vTakenFrame.push_back(time);
		}
		
		vCamera[iCamera].vTakenFrame.pop_back();
		fin.close();
	}
	int min_time = 1e+10;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		if (vCamera[iCamera].vTakenFrame[0] < min_time)
			min_time = vCamera[iCamera].vTakenFrame[0];
	}
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			vCamera[iCamera].vTakenFrame[iFrame] -= min_time;
		}
	}
}

void LoadExifDataWithFocalLength(vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Exif Data");
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		cout << "File name: " << vCamera[iCamera].exifFileName << endl;
		ifstream fin;
		fin.open(vCamera[iCamera].exifFileName.c_str(), ios_base::in);
		int iFrame = 0;
		vCamera[iCamera].vTakenFrame.clear();
		while (!fin.eof())
		{
			string a;
			int time;
			double focalLength;
			double px, py;
			fin >> a;
			fin >> time >> focalLength >> px >> py;
			time = time - vCamera[iCamera].timeOffset;
			vCamera[iCamera].vTakenFrame.push_back(time);	
			vCamera[iCamera].vFocalLength.push_back(focalLength);
			vCamera[iCamera].vpx.push_back(px);
			vCamera[iCamera].vpy.push_back(py);
		}
		vCamera[iCamera].vTakenFrame.pop_back();
		vCamera[iCamera].vFocalLength.pop_back();
		vCamera[iCamera].vpx.pop_back();
		vCamera[iCamera].vpy.pop_back();
		fin.close();
	}
	int min_time = 1e+10;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		if (vCamera[iCamera].vTakenFrame[0] < min_time)
			min_time = vCamera[iCamera].vTakenFrame[0];
	}
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			vCamera[iCamera].vTakenFrame[iFrame] -= min_time;

			CvMat *K = cvCreateMat(3,3,CV_32FC1);
			cvSetIdentity(K);
			cvSetReal2D(K, 0, 0, vCamera[iCamera].vpx[iFrame]*vCamera[iCamera].vFocalLength[iFrame]/vCamera[iCamera].ccd_width);
			cvSetReal2D(K, 1, 1, vCamera[iCamera].vpx[iFrame]*vCamera[iCamera].vFocalLength[iFrame]/vCamera[iCamera].ccd_width);
			cvSetReal2D(K, 0, 2, vCamera[iCamera].vpx[iFrame]/2);
			cvSetReal2D(K, 1, 2, vCamera[iCamera].vpy[iFrame]/2);

			vCamera[iCamera].vK.push_back(K);
			vCamera[iCamera].vk1.push_back(0);
			vCamera[iCamera].vk2.push_back(0);
		}
	}
}

void LoadExifDataWithFocalLengthWOK(vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Exif Data");
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		cout << "File name: " << vCamera[iCamera].exifFileName << endl;
		ifstream fin;
		fin.open(vCamera[iCamera].exifFileName.c_str(), ios_base::in);
		int iFrame = 0;
		vCamera[iCamera].vTakenFrame.clear();
		while (!fin.eof())
		{
			string a;
			int time;
			double focalLength;
			double px, py;
			fin >> a;
			fin >> time >> focalLength >> px >> py;
			time = time - vCamera[iCamera].timeOffset;
			vCamera[iCamera].vTakenFrame.push_back(time);	
			vCamera[iCamera].vFocalLength.push_back(focalLength);
			vCamera[iCamera].vpx.push_back(px);
			vCamera[iCamera].vpy.push_back(py);
		}
		vCamera[iCamera].vTakenFrame.pop_back();
		vCamera[iCamera].vFocalLength.pop_back();
		vCamera[iCamera].vpx.pop_back();
		vCamera[iCamera].vpy.pop_back();
		fin.close();
	}
	int min_time = 1e+10;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		if (vCamera[iCamera].vTakenFrame[0] < min_time)
			min_time = vCamera[iCamera].vTakenFrame[0];
	}
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			vCamera[iCamera].vTakenFrame[iFrame] -= min_time;

			CvMat *K = cvCreateMat(3,3,CV_32FC1);
			cvSetIdentity(K);
			//cvSetReal2D(K, 0, 0, vCamera[iCamera].vpx[iFrame]*vCamera[iCamera].vFocalLength[iFrame]/vCamera[iCamera].ccd_width);
			//cvSetReal2D(K, 1, 1, vCamera[iCamera].vpx[iFrame]*vCamera[iCamera].vFocalLength[iFrame]/vCamera[iCamera].ccd_width);
			//cvSetReal2D(K, 0, 2, vCamera[iCamera].vpx[iFrame]/2);
			//cvSetReal2D(K, 1, 2, vCamera[iCamera].vpy[iFrame]/2);

			vCamera[iCamera].vK.push_back(K);
			vCamera[iCamera].vk1.push_back(0);
			vCamera[iCamera].vk2.push_back(0);
		}
	}
}


void LoadCalibrationData(vector<Camera> &vCamera)
{
	PrintAlgorithm("Load Calibration Data");

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		cout << "File name: " << vCamera[iCamera].calibrationFilename << endl;
		ifstream file;
		string a;
		int nCameras;
		file.open(vCamera[iCamera].calibrationFilename.c_str(), ifstream::in);
		file >> a;
		CvMat *K;
		double k1, k2;
		double k11, k12, k13, k21, k22, k23, k31, k32, k33;
		file >> k11 >> k12 >> k13;
		file >> k21 >> k22 >> k23;
		file >> k31 >> k32 >> k33;
		file >> k1 >> k2;

		K = cvCreateMat(3,3,CV_32FC1);
		cvSetReal2D(K, 0, 0, k11);		cvSetReal2D(K, 0, 1, k12);		cvSetReal2D(K, 0, 2, k13);
		cvSetReal2D(K, 1, 0, k21);		cvSetReal2D(K, 1, 1, k22);		cvSetReal2D(K, 1, 2, k23);
		cvSetReal2D(K, 2, 0, k31);		cvSetReal2D(K, 2, 1, k32);		cvSetReal2D(K, 2, 2, k33);
		vCamera[iCamera].K = K;
		vCamera[iCamera].k1 = k1;
		vCamera[iCamera].k2 = k2;
		file.close();
	}
}

void LoadCalibrationData(vector<Camera> &vCamera, vector<int> vWOCalibrationCameraID)
{
	PrintAlgorithm("Load Calibration Data");

	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		vector<int> ::const_iterator it = find(vWOCalibrationCameraID.begin(), vWOCalibrationCameraID.end(), vCamera[iCamera].id);
		if (it == vWOCalibrationCameraID.end())
			continue;
		cout << "File name: " << vCamera[iCamera].calibrationFilename << endl;
		ifstream file;
		string a;
		int nCameras;
		file.open(vCamera[iCamera].calibrationFilename.c_str(), ifstream::in);
		file >> a;
		CvMat *K;
		double k1, k2;
		double k11, k12, k13, k21, k22, k23, k31, k32, k33;
		file >> k11 >> k12 >> k13;
		file >> k21 >> k22 >> k23;
		file >> k31 >> k32 >> k33;
		file >> k1 >> k2;

		K = cvCreateMat(3,3,CV_32FC1);
		cvSetReal2D(K, 0, 0, k11);		cvSetReal2D(K, 0, 1, k12);		cvSetReal2D(K, 0, 2, k13);
		cvSetReal2D(K, 1, 0, k21);		cvSetReal2D(K, 1, 1, k22);		cvSetReal2D(K, 1, 2, k23);
		cvSetReal2D(K, 2, 0, k31);		cvSetReal2D(K, 2, 1, k32);		cvSetReal2D(K, 2, 2, k33);
		vCamera[iCamera].K = K;
		vCamera[iCamera].k1 = 0;
		vCamera[iCamera].k2 = 0;
		vCamera[iCamera].vK.clear();
		vCamera[iCamera].vk1.clear();
		vCamera[iCamera].vk2.clear();
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			CvMat *K_ = cvCreateMat(3,3,CV_32FC1);
			K_ = cvCloneMat(K);
			vCamera[iCamera].vK.push_back(K_);
			vCamera[iCamera].vk1.push_back(0);
			vCamera[iCamera].vk2.push_back(0);
		}
		file.close();
	}
}

void SaveMatrix(string filename, CvMat *M)
{
	PrintAlgorithm("Save Matrix");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << M->rows << " " << M->cols << endl;
	
	for (int irow = 0; irow < M->rows; irow++)
	{
		for (int icol = 0; icol < M->cols; icol++)
		{
			fout << cvGetReal2D(M, irow, icol) << " ";
		}
		fout << endl;
	}

	fout.close();
}
