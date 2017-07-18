#include "DataUtility.h"

void SaveFaceReconstructionData(string filename, vector<vector<FaceReconstruction> > vvFaceReconstruction)
{
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "nFrames: " << vvFaceReconstruction.size() << endl;
	for (int iTime = 0; iTime < vvFaceReconstruction.size(); iTime++)
	{
		fout << vvFaceReconstruction[iTime].size() << endl;
		
		for (int iFace = 0; iFace < vvFaceReconstruction[iTime].size(); iFace++)
		{
			fout << vvFaceReconstruction[iTime][iFace].id << " " << vvFaceReconstruction[iTime][iFace].vPanel.size() << " ";
			fout << vvFaceReconstruction[iTime][iFace].vX.size() << " ";
			fout << vvFaceReconstruction[iTime][iFace].px << " ";
			fout << vvFaceReconstruction[iTime][iFace].py << " ";
			fout << vvFaceReconstruction[iTime][iFace].pz << " ";

			fout << vvFaceReconstruction[iTime][iFace].nx << " ";
			fout << vvFaceReconstruction[iTime][iFace].ny << " ";
			fout << vvFaceReconstruction[iTime][iFace].nz << " ";

			for (int ix = 0; ix < vvFaceReconstruction[iTime][iFace].vX.size(); ix++)
			{
				fout << vvFaceReconstruction[iTime][iFace].vX[ix] << " ";
				fout << vvFaceReconstruction[iTime][iFace].vY[ix] << " ";
				fout << vvFaceReconstruction[iTime][iFace].vZ[ix] << " ";
			}

			for (int ip = 0; ip < vvFaceReconstruction[iTime][iFace].vCamera.size(); ip++)
			{
				fout << vvFaceReconstruction[iTime][iFace].vPanel[ip] << " ";
				fout << vvFaceReconstruction[iTime][iFace].vCamera[ip] << " ";
			}
			fout << endl;
		}
	}
	fout.close();
}

void LoadFaceRefinedMeasurementData(string filename, vector<vector<FaceReconstruction> > &vvFaceReconstruction)
{
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), std::ifstream::in);
	int nFrames;
	string dummy;
	fin >> dummy >> nFrames;

	for (int iFrame = 0; iFrame < nFrames; iFrame++)
	{
		vector<FaceReconstruction> vFaceReconstruction;
		int nFaces;
		fin >> nFaces;
		for (int iFace = 0; iFace < nFaces; iFace++)
		{
			FaceReconstruction fr;
			int id, np;
			double x, y, z;
			fin >> id >> np;
			fr.id = id;
			fin >> fr.px >> fr.py >> fr.pz;
			fin >> fr.nx >> fr.ny >> fr.nz;

			for (int ip = 0; ip < np; ip++)
			{
				fin >> x >> y >> z;
				fr.vX.push_back(x);
				fr.vY.push_back(y);
				fr.vZ.push_back(z);
			}

			int nM;
			fin >> dummy >> nM;
			for (int iM = 0; iM < nM; iM++)
			{
				int panel, camera;
				fin >> panel >> camera;
				fr.vPanel.push_back(panel);
				fr.vCamera.push_back(camera);
				vector<double> vu, vv;
				for (int ip = 0; ip < np; ip++)
				{
					fin >> x >> y;
					vu.push_back(x);
					vv.push_back(y);
				}
				fr.vvu.push_back(vu);
				fr.vvv.push_back(vv);
			}
			vFaceReconstruction.push_back(fr);
		}
		vvFaceReconstruction.push_back(vFaceReconstruction);		
	}
	fin.close();
}

void LoadSocialFaceData(string filename, vector<vector<double> > &vvx, vector<vector<double> > &vvy, vector<vector<double> > &vvz)
{
	std::ifstream fin;
	fin.open(filename.c_str(), std::ifstream::in);

	string dummy;
	int nFrames;
	double bandwidth;
	fin >> dummy >> nFrames;
	fin >> dummy >> bandwidth;

	for (int iFrame = 0; iFrame < nFrames; iFrame++)
	{
		vector<double> vx, vy, vz;
		int nFaces;
		fin >> nFaces;
		for (int iFace = 0; iFace < nFaces; iFace++)
		{
			int id;
			double px, py, pz, nx, ny, nz;
			fin >> id;
			fin >> px >> py >> pz >> nx >> ny >> nz;
		}
		int nPOI;
		fin >> nPOI;
		for (int i = 0; i < nPOI; i++)
		{
			double x, y, z;
			fin >> x >> y >> z;
			vx.push_back(x);
			vy.push_back(y);
			vz.push_back(z);
		}
		vvx.push_back(vx);
		vvy.push_back(vy);
		vvz.push_back(vz);
	}


	fin.close();
}

void LoadFaceData(string filename, vector<FaceReconstruction> &vFaceReconstruction)
{
	std::ifstream fin;
	fin.open(filename.c_str(), std::ifstream::in);

	string dummy;
	int nFaces;
	fin >> dummy >> nFaces;

	for (int iFace = 0; iFace < nFaces; iFace++)
	{
		FaceReconstruction face;
		int nProjections;
		int nPoints;
		fin >> nProjections >> nPoints;

		fin >> face.px >> face.py >> face.pz;
		fin >> face.nx >> face.ny >> face.nz;
		for (int i = 0; i < nPoints; i++)
		{
			double x, y, z;
			fin >> x >> y >> z;
			face.vX.push_back(x);
			face.vY.push_back(y);
			face.vZ.push_back(z);
		}

		for (int i = 0; i < nProjections; i++)
		{
			int panel, camera;
			fin >> panel >> camera;
			face.vPanel.push_back(panel);
			face.vCamera.push_back(camera);
		}
		vFaceReconstruction.push_back(face);
	}

	fin.close();
}

void LoadDomeCameraData(string intrinsic, string extrinsic, DomeCamera &dome_camera)
{
	PrintAlgorithm("Load Dome Camera Data");
	cout << "File name: " << intrinsic << endl;
	ifstream file;
	file.open(intrinsic.c_str(), ifstream::in);

	double k11, k12, k13, k21, k22, k23, k31, k32, k33, dist1, dist2;
	file >> k11 >> k12 >> k13 >> k21 >> k22 >> k23 >> k31 >> k32 >> k33 >> dist1 >> dist2;
	dome_camera.K = cvCreateMat(3,3,CV_32FC1);
	cvSetReal2D(dome_camera.K, 0, 0, k11);	cvSetReal2D(dome_camera.K, 0, 1, k12);	cvSetReal2D(dome_camera.K, 0, 2, k13);
	cvSetReal2D(dome_camera.K, 1, 0, k21);	cvSetReal2D(dome_camera.K, 1, 1, k22);	cvSetReal2D(dome_camera.K, 1, 2, k23);
	cvSetReal2D(dome_camera.K, 2, 0, k31);	cvSetReal2D(dome_camera.K, 2, 1, k32);	cvSetReal2D(dome_camera.K, 2, 2, k33);
	dome_camera.dist1 = dist1;
	dome_camera.dist2 = dist2;

	file.close();

	file.open(extrinsic.c_str(), ifstream::in);

	double q1, q2, q3, q4, c1, c2, c3;
	file >> q1 >> q2 >> q3 >> q4 >> c1 >> c2 >> c3;
	dome_camera.C = cvCreateMat(3,1,CV_32FC1);
	cvSetReal2D(dome_camera.C, 0, 0, c1);	cvSetReal2D(dome_camera.C, 1, 0, c2);	cvSetReal2D(dome_camera.C, 2, 0, c3);
	CvMat *q = cvCreateMat(4,1,CV_32FC1);
	cvSetReal2D(q, 0, 0, q1);
	cvSetReal2D(q, 1, 0, q2);
	cvSetReal2D(q, 2, 0, q3);
	cvSetReal2D(q, 3, 0, q4);

	dome_camera.R = cvCreateMat(3,3,CV_32FC1);
	Quaternion2Rotation(q, dome_camera.R);

	file.close();

	dome_camera.P = cvCreateMat(3,4,CV_32FC1);
	cvSetIdentity(dome_camera.P);
	CvMat *C_minus = cvCreateMat(3,1,CV_32FC1);
	ScalarMul(dome_camera.C, -1, C_minus);
	SetSubMat(dome_camera.P, 0, 3, C_minus);
	cvMatMul(dome_camera.R, dome_camera.P, dome_camera.P);
	cvMatMul(dome_camera.K, dome_camera.P, dome_camera.P);

	cvReleaseMat(&C_minus);
	cvReleaseMat(&q);
}

void LoadFaceDetectionData(string filename, vector<Face> &vFace)
{
	PrintAlgorithm("Load Face Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);

	int nFaces;
	file >> nFaces;
	for (int i = 0; i < nFaces; i++)
	{
		Face face;
		int n;
		file >> face.panel >> face.camera >> n;
		for (int j = 0; j < n; j++)
		{
			double x, y;
			file >> x >> y;
			face.vx.push_back(x);
			face.vy.push_back(y);
		}
		vFace.push_back(face);
	}

	file.close();
}

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

void SavePOI_Matches(string filename, vector<POI_Matches> vPOI_matches)
{
	PrintAlgorithm("Save POI Match Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	if (vPOI_matches[0].vvWeight.size() == 0)
		fout << "NumCameras: 0" << endl;
	else
		fout << "NumCameras: " << vPOI_matches[0].vvWeight[0].size() << endl;
	fout << "NumMatches: " << vPOI_matches.size() << endl;
	for (int iMatch = 0; iMatch < vPOI_matches.size(); iMatch++)
	{
		fout << vPOI_matches[iMatch].vx.size() << " ";
		for (int iFrame = 0; iFrame < vPOI_matches[iMatch].vx.size(); iFrame++)
		{
			fout << vPOI_matches[iMatch].vFrame[iFrame] << " ";
			fout << vPOI_matches[iMatch].vx[iFrame] << " ";
			fout << vPOI_matches[iMatch].vy[iFrame] << " ";
			fout << vPOI_matches[iMatch].vz[iFrame] << " ";
			for (int iW = 0; iW < vPOI_matches[iMatch].vvWeight[iFrame].size(); iW++)
			{
				fout << vPOI_matches[iMatch].vvWeight[iFrame][iW] << " ";
			}			
		}	
		fout << endl;
	}
	fout.close();
}

void SavePOI_Matches_light(string filename, vector<POI_Matches> vPOI_matches)
{
	PrintAlgorithm("Save POI Match Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumMatches: " << vPOI_matches.size() << endl;
	for (int iMatch = 0; iMatch < vPOI_matches.size(); iMatch++)
	{
		fout << vPOI_matches[iMatch].vx.size() << " ";
		for (int iFrame = 0; iFrame < vPOI_matches[iMatch].vx.size(); iFrame++)
		{
			fout << vPOI_matches[iMatch].vFrame[iFrame] << " ";
			fout << vPOI_matches[iMatch].vx[iFrame] << " ";
			fout << vPOI_matches[iMatch].vy[iFrame] << " ";
			fout << vPOI_matches[iMatch].vz[iFrame] << " ";			
		}	
		fout << endl;
	}
	fout.close();
}

void LoadPOI_Matches_light(string filename, vector<POI_Matches> &vPOI_matches)
{
	PrintAlgorithm("Load POI Match Data");
	cout << "File name: " << filename << endl;

	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string dummy;
	int nMatches;
	fin >> dummy >> nMatches;
	for (int iMatch = 0; iMatch < nMatches; iMatch++)
	{
		int nPoint;
		fin >> nPoint;
		POI_Matches poi;
		for (int iPoint = 0; iPoint < nPoint; iPoint++)
		{
			int frame;
			double x, y, z;
			fin >> frame >> x >> y >> z;
			poi.vFrame.push_back(frame);
			poi.vx.push_back(x);
			poi.vy.push_back(y);
			poi.vz.push_back(z);
		}
		vPOI_matches.push_back(poi);
	}
	fin.close();
}


void LoadPOIMatchData(string filename, vector<POI_Matches> &vPOI)
{
	PrintAlgorithm("Load POI Match Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nCameras, nPOI;
	fin >> a >> nCameras;
	fin >> a >> nPOI;
	for (int i = 0; i < nPOI; i++)
	{
		POI_Matches poi;
		int nFrames, frame;
		double x, y, z;
		double dummy;
		fin >> nFrames;
		for (int j = 0; j < nFrames; j++)
		{
			fin >> frame >> x >> y >> z;
			vector<double> vWeight;
			for (int k = 0; k < nCameras; k++)
			{
				fin >> dummy;
				vWeight.push_back(dummy);
			}
			poi.vFrame.push_back(frame);
			poi.vx.push_back(x);
			poi.vy.push_back(y);
			poi.vz.push_back(z);
			poi.vvWeight.push_back(vWeight);
		}
		vPOI.push_back(poi);
	}
	fin.close();
}


void LoadCameraTrajectoryData(string filename, vector<CvMat*> &vC, vector<CvMat*> &vR)
{
	PrintAlgorithm("Load Camera Trajectory Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> nCameras;

	for (int i = 0; i < nCameras; i++)
	{
		CvMat *C = cvCreateMat(3,1,CV_32FC1);
		CvMat *R = cvCreateMat(3,3,CV_32FC1);

		double iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33, timeInstant;
		int takenFrame;
		fin >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		cvSetReal2D(C, 0,0, t1);	cvSetReal2D(C, 1,0, t2);	cvSetReal2D(C, 2,0, t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);

		vC.push_back(C);
		vR.push_back(R);
	}	

	fin.close();
}

void LoadCameraTrajectoryData(string filename, vector<CvMat*> &vC, vector<CvMat*> &vR, int &endframe)
{
	PrintAlgorithm("Load Camera Trajectory Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> endframe >> nCameras;

	for (int i = 0; i < nCameras; i++)
	{
		CvMat *C = cvCreateMat(3,1,CV_32FC1);
		CvMat *R = cvCreateMat(3,3,CV_32FC1);

		double iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33, timeInstant;
		int takenFrame;
		fin >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		cvSetReal2D(C, 0,0, t1);	cvSetReal2D(C, 1,0, t2);	cvSetReal2D(C, 2,0, t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);

		vC.push_back(C);
		vR.push_back(R);
	}	

	fin.close();
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
		double x, y, scale, orienation;
		file >> y >> x >> scale >> orienation;
		vector<int> desc;
		for (int iDim = 0; iDim < nDim; iDim++)
		{
			double dummy;
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
		descriptor.orientation = orienation;

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

void LoadStaticMeasurementData_NoCalibration_RGB(string filename, vector<Feature> &vFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	file.open(filename.c_str(), ifstream::in);
	while (!file.eof())
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >> b;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0;
		feature.r = r;
		feature.g = g;
		feature.b = b;
		feature.id = id;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x,y,x0,y0;
			file >> cam >> frame;
			file >> x >> y >> x0 >> y0;
			feature.vx.push_back(x);
			feature.vy.push_back(y);
			feature.vx_dis.push_back(x);
			feature.vy_dis.push_back(y);
			feature.vCamera.push_back(cam);
			feature.vFrame.push_back(frame);
		}
		vFeature.push_back(feature);
	}
	vFeature.pop_back();
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

void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast_SO(string filename, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras, nFeatures;
	int nFrames;
	file >> a >> nCameras;
	file >> a >> nFrames;
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
		vector<double> vx, vy, vx0, vy0, vx_dis, vy_dis;
		double x, y, dis_x, dis_y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y >> dis_x >> dis_y;
			x0 = x;
			y0 = y;

			vCam.push_back(cam);
			vFrame.push_back(frame);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vx_dis.push_back(dis_x);
			vy_dis.push_back(dis_y);
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
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		feature.isRegistered = false;

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
	if (vCamera.empty())
	{
		for (int iCamera = 0; iCamera < nCameras; iCamera++)
		{
			Camera cam;
			cam.id = iCamera;
			vCamera.push_back(cam);
		}

		for (int iCamera = 0; iCamera < nCameras; iCamera++)
		{
			int nFrames;
			file >> nFrames;
			for (int iFrame = 0; iFrame < nFrames; iFrame++)
			{
				vCamera[iCamera].vTakenFrame.push_back(iFrame);
			}
			vCamera[iCamera].nFrames = vCamera[iCamera].vTakenFrame.size();
		}
	}
	else
	{
		for (int iCamera = 0; iCamera < nCameras; iCamera++)
		{
			int nFrames;
			file >> nFrames;
			vCamera[iCamera].nFrames = vCamera[iCamera].vTakenFrame.size();
		}
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
		vector<double> vx, vy, vx0, vy0, vx_dis, vy_dis;
		double x, y, dis_x, dis_y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y >> dis_x >> dis_y;
			x0 = x;
			y0 = y;

			bool isIn = false;
			for (int iCam = 0; iCam < vCam.size(); iCam++)
			{
				for (int iFr = 0; iFr < vFrame.size(); iFr++)
				{
					if ((vCam[iCam] == cam) && (vFrame[iFr] == frame))
						isIn = true;
				}
			}
			if (isIn)
				continue;

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
			vx_dis.push_back(dis_x);
			vy_dis.push_back(dis_y);
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
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		feature.isRegistered = false;

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;

		vUncalibratedFeature.push_back(feature);
	}
	file.close();
}

void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast(string filename, vector<Feature> &vFeature)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras, nFeatures;
	
	while (!file.eof())
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >> b;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0, vx_dis, vy_dis;
		double x, y, dis_x, dis_y;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y >> dis_x >> dis_y;
			//file >> y >> x >> dis_y >> dis_x;
			x0 = x;
			y0 = y;

			vCam.push_back(cam);
			vFrame.push_back(frame);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vx_dis.push_back(dis_x);
			vy_dis.push_back(dis_y);
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
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		feature.isRegistered = false;

		vFeature.push_back(feature);
		feature.vx = vx0;
		feature.vy = vy0;
		feature.vFrame = vFrame0;
	}
	vFeature.pop_back();
	file.close();
}

void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast_InitialFrame(string filename, vector<Feature> &vFeature, int &max_frame)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nCameras=1, nFeatures;

	max_frame = 0;

	while (!file.eof())
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >> b;
		Feature feature;	
		vector<double> vx, vy, vx0, vy0, vx_dis, vy_dis;
		double x, y, dis_x, dis_y;
		vector<int> vCam, vFrame;

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y >> dis_x >> dis_y;
			x0 = x;
			y0 = y;

			if (max_frame < frame)
				max_frame = frame;

			vCam.push_back(cam);
			vFrame.push_back(frame);
			vx.push_back(x);	vy.push_back(y);
			vx0.push_back(x0);	vy0.push_back(y0);
			vx_dis.push_back(dis_x);
			vy_dis.push_back(dis_y);
		}
		feature.id = id;
		feature.r = r; 
		feature.g = g;
		feature.b = b;
		feature.vFrame = vFrame;
		feature.vCamera = vCam;
		feature.vx = vx;
		feature.vy = vy;
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		feature.isRegistered = false;

		vFeature.push_back(feature);
	}
	vFeature.pop_back();
	max_frame++;

	file.close();
}

void SaveStaticMeasurementData_NoCalibration_RGB_LOWES_fast_InitialFrame(string filename, vector<Feature> vFeature, int max_frame)
{
	PrintAlgorithm("Save Measurement Data");
	cout << "File name: " << filename << endl;
	ofstream file;
	string a;
	file.open(filename.c_str(), ifstream::out);
	int nCameras=1, nFeatures;
	file << "nCameras: 1" << endl;
	file << "nFrames: " << max_frame << endl;
	file << "nPoints: " << vFeature.size() << endl;

	for (int iFeature = 0; iFeature < vFeature.size(); iFeature++)
	{
		file << vFeature[iFeature].vFrame.size() << " " << vFeature[iFeature].id << " ";
		file << vFeature[iFeature].r << " " << vFeature[iFeature].g << " " << vFeature[iFeature].b << " ";

		for (int iFrame = 0; iFrame < vFeature[iFeature].vFrame.size(); iFrame++)
		{
			file << vFeature[iFeature].vCamera[iFrame] << " " << vFeature[iFeature].vFrame[iFrame] << " ";
			file << vFeature[iFeature].vx[iFrame] << " " << vFeature[iFeature].vy[iFrame] << " ";
			file << vFeature[iFeature].vx_dis[iFrame] << " " << vFeature[iFeature].vy_dis[iFrame] << " ";
		}
		file << endl;
	}

	file.close();
}

void SaveInitialFrame(string filename, int init1, int init2, int max_frame)
{
	PrintAlgorithm("Save InitialFrame Data");
	cout << "File name: " << filename << endl;
	ofstream file;
	string a;
	file.open(filename.c_str(), ifstream::out);
	file << init1 << " " << init2 << " " << max_frame << endl;
	file.close();
}

void SaveInitialFrameList(string filename, vector<int> init1, vector<int> init2, vector<double> vdist, 
							vector<double> vportion, vector<int> nCorr)
{
	PrintAlgorithm("Save InitialFrame Data");
	cout << "File name: " << filename << endl;
	ofstream file;
	string a;
	file.open(filename.c_str(), ifstream::out);
	for (int i = 0; i < init1.size(); i++)
	{
		file << init1[i] << " " << init2[i] << " " << vportion[i] << " " << vdist[i] << " " << nCorr[i] << endl;
	}
	
	file.close();
}


void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast_Interpolation(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, int &max_nFrames)
{
	PrintAlgorithm("Load Measurement Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);

	for (int iCamera = 0; iCamera < 1; iCamera++)
	{
		Camera cam;
		cam.id = iCamera;
		vCamera.push_back(cam);
	}

	max_nFrames = 0;
	while (!file.eof())
	{
		int nVisibleFrame;
		int id;
		double r, g, b;
		file >> nVisibleFrame >> id;
		file >> r >> g >> b;
		Feature feature;	
		vector<int> vFrame, vCam, vFrame0;
		vector<double> vx, vy, vx0, vy0, vx_dis, vy_dis;
		double x, y, dis_x, dis_y;

		if (file.eof())
		{
			break;
		}

		for (int i = 0; i < nVisibleFrame; i++)
		{	
			int frame, cam;			
			double x0,y0;
			file >> cam >> frame;
			file >> x >> y >> dis_x >> dis_y;
			x0 = x;
			y0 = y;

			vector<int>::iterator it = find(vCamera[0].vTakenFrame.begin(), vCamera[0].vTakenFrame.end(), frame);
			if (it == vCamera[0].vTakenFrame.end())
			{
				vCamera[0].vTakenFrame.push_back(frame);
				if (frame > max_nFrames)
				{
					max_nFrames = frame;
				}
			}

			vx.push_back(x);	vy.push_back(y);
			vx_dis.push_back(dis_x);
			vy_dis.push_back(dis_y);
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
		feature.vx_dis = vx_dis;
		feature.vy_dis = vy_dis;
		feature.isRegistered = false;

		vFeature.push_back(feature);
	}
	file.close();
	max_nFrames++;
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
		vector<vector<int> > vvDesc;

		for (int i = 0; i < nVisibleFrame; i++)
		{			
			int temp, frame, cam;

			double x, y;

			file >> cam >> frame >> x >> y;

			vector<int> vDesc;
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
		vector<int> vMeanDesc;
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

void LoadCameraData(string filename, Camera &camera)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	fin >> nFrames;
	fin >> a >> nTotalFrames;

	for (int i = 0; i < nTotalFrames; i++)
	{
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *P = cvCreateMat(3,4,CV_32FC1);
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
		cvSetIdentity(P);
		SetSubMat(P, 0, 3, t);
		cvMatMul(R, P, P);
		cvMatMul(camera.K, P, P);

		camera.vP.push_back(P);
		camera.vTakenFrame.push_back(iFrame);
		camera.vTakenInstant.push_back(iFrame);
		camera.vR.push_back(R);
		ScalarMul(t, -1, t);
		camera.vC.push_back(t);
	}	

	fin.close();
}

void LoadCameraData(string filename, Camera &camera, int &nFrames)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	fin >> nFrames;
	fin >> a >> nTotalFrames;

	for (int i = 0; i < nTotalFrames; i++)
	{
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *P = cvCreateMat(3,4,CV_32FC1);
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
		cvSetIdentity(P);
		SetSubMat(P, 0, 3, t);
		cvMatMul(R, P, P);
		cvMatMul(camera.K, P, P);

		camera.vP.push_back(P);
		camera.vTakenFrame.push_back(iFrame);
		camera.vTakenInstant.push_back(iFrame);
		camera.vR.push_back(R);
		ScalarMul(t, -1, t);
		camera.vC.push_back(t);
	}	

	fin.close();
}

void LoadCameraData_AD(string filename, Camera &camera, int &nFrames, vector<vector<int> > &vvPointIdx)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	fin >> nFrames;
	fin >> a >> nTotalFrames;

	for (int i = 0; i < nTotalFrames; i++)
	{
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *P = cvCreateMat(3,4,CV_32FC1);
		double iCamera, iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33;
		fin >> iCamera >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;

		int nPoints;
		fin >> nPoints;
		vector<int> vPointIdx;
		for (int iPoint = 0; iPoint < nPoints; iPoint++)
		{
			int idx;
			fin >> idx;
			vPointIdx.push_back(idx);
		}
		cvSetReal2D(t, 0,0, -t1);	cvSetReal2D(t, 1,0, -t2);	cvSetReal2D(t, 2,0, -t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);
		cvSetIdentity(P);
		SetSubMat(P, 0, 3, t);
		cvMatMul(R, P, P);
		cvMatMul(camera.K, P, P);

		camera.vP.push_back(P);
		camera.vTakenFrame.push_back(iFrame);
		camera.vTakenInstant.push_back(iFrame);
		camera.vR.push_back(R);
		ScalarMul(t, -1, t);
		camera.vC.push_back(t);
		vvPointIdx.push_back(vPointIdx);
	}	

	fin.close();
}

void LoadCameraData_Ray(string filename, Camera &camera)
{
	PrintAlgorithm("Load Camera Ray Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nFrames, nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	fin >> nFrames;
	fin >> a >> nTotalFrames;
	double bandwidth;
	fin >> a >> bandwidth;
	camera.rayBandwidth = bandwidth;

	for (int i = 0; i < nTotalFrames; i++)
	{
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *P = cvCreateMat(3,4,CV_32FC1);
		CvMat *rayCenter = cvCreateMat(3,1, CV_32FC1);
		CvMat *rayDirection = cvCreateMat(3,1, CV_32FC1);
		double iCamera, iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33;
		double rayC1, rayC2, rayC3, rayD1, rayD2, rayD3;
		fin >> iCamera >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		fin >> rayC1 >> rayC2 >> rayC3;
		fin >> rayD1 >> rayD2 >> rayD3;
		cvSetReal2D(t, 0,0, -t1);	cvSetReal2D(t, 1,0, -t2);	cvSetReal2D(t, 2,0, -t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);
		cvSetIdentity(P);
		SetSubMat(P, 0, 3, t);
		cvMatMul(R, P, P);
		cvMatMul(camera.K, P, P);

		cvSetReal2D(rayCenter, 0, 0, rayC1);	cvSetReal2D(rayCenter, 1, 0, rayC2);	cvSetReal2D(rayCenter, 2, 0, rayC3);
		cvSetReal2D(rayDirection, 0, 0, rayD1);	cvSetReal2D(rayDirection, 1, 0, rayD2);	cvSetReal2D(rayDirection, 2, 0, rayD3);

		camera.vP.push_back(P);
		camera.vTakenFrame.push_back(iFrame);
		camera.vTakenInstant.push_back(iFrame);
		camera.vR.push_back(R);
		camera.vRayCenter.push_back(rayCenter);
		camera.vRayDirection.push_back(rayDirection);
		ScalarMul(t, -1, t);
		camera.vC.push_back(t);
	}	

	fin.close();
}

void LoadCameraData_Ray(string filename, Camera &camera, int &nFrames)
{
	PrintAlgorithm("Load Camera Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int nCameras, nTotalFrames;
	string a;
	fin >> a >> nCameras;
	fin >> a;
	fin >> nFrames;
	fin >> a >> nTotalFrames;

	for (int i = 0; i < nTotalFrames; i++)
	{
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		CvMat *t = cvCreateMat(3,1,CV_32FC1);
		CvMat *P = cvCreateMat(3,4,CV_32FC1);
		CvMat *C = cvCreateMat(3,1,CV_32FC1);
		CvMat *rayCenter = cvCreateMat(3,1, CV_32FC1);
		CvMat *rayDirection = cvCreateMat(3,1, CV_32FC1);
		double iCamera, iFrame, t1, t2, t3, r11, r12, r13, r21, r22, r23, r31, r32, r33;
		double rayC1, rayC2, rayC3, rayD1, rayD2, rayD3;
		fin >> iCamera >> iFrame;
		fin >> t1 >> t2 >> t3;
		fin >> r11 >> r12 >> r13;
		fin >> r21 >> r22 >> r23;
		fin >> r31 >> r32 >> r33;
		fin >> rayC1 >> rayC2 >> rayC3;
		fin >> rayD1 >> rayD2 >> rayD3;
		cvSetReal2D(t, 0,0, -t1);	cvSetReal2D(t, 1,0, -t2);	cvSetReal2D(t, 2,0, -t3);
		cvSetReal2D(R, 0,0, r11);	cvSetReal2D(R, 0,1, r12);	cvSetReal2D(R, 0,2, r13);
		cvSetReal2D(R, 1,0, r21);	cvSetReal2D(R, 1,1, r22);	cvSetReal2D(R, 1,2, r23);
		cvSetReal2D(R, 2,0, r31);	cvSetReal2D(R, 2,1, r32);	cvSetReal2D(R, 2,2, r33);
		cvSetIdentity(P);
		SetSubMat(P, 0, 3, t);
		cvMatMul(R, P, P);
		cvMatMul(camera.K, P, P);
		ScalarMul(t, -1, C);

		cvSetReal2D(rayCenter, 0, 0, rayC1);	cvSetReal2D(rayCenter, 1, 0, rayC2);	cvSetReal2D(rayCenter, 2, 0, rayC3);
		cvSetReal2D(rayDirection, 0, 0, rayD1);	cvSetReal2D(rayDirection, 1, 0, rayD2);	cvSetReal2D(rayDirection, 2, 0, rayD3);

		camera.vP.push_back(P);
		camera.vTakenFrame.push_back(iFrame);
		camera.vTakenInstant.push_back(iFrame);
		camera.vR.push_back(R);
		camera.vC.push_back(C);
		camera.vRayCenter.push_back(rayCenter);
		camera.vRayDirection.push_back(rayDirection);
		ScalarMul(t, -1, t);
		camera.vC.push_back(t);
	}	

	fin.close();
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

void CreateEmptyFile(string filename)
{
	PrintAlgorithm("Create Empty File");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout.close();
}

void SavePOI_Meanshift(string filename, vector<vector<CvMat *> > vvPOI, int nSegments_cov, vector<vector<CvMat *> > vv_a, vector<vector<CvMat *> > vv_b, vector<vector<CvMat *> > vv_l, vector<vector<double> > vvf, vector<int> vFrame)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumFrames: " << vvPOI.size() << endl;
	fout << "NumSegments_Cov: " << nSegments_cov << endl;
	for (int iFrame = 0; iFrame < vFrame.size(); iFrame++)
	{
		vector<CvMat *> vPOI = vvPOI[iFrame];
		vector<CvMat *> v_a = vv_a[iFrame];
		vector<CvMat *> v_b = vv_b[iFrame];
		vector<CvMat *> v_l = vv_l[iFrame];
		vector<double> vf = vvf[iFrame];
		fout << vFrame[iFrame] << " " << vPOI.size() << " ";
		for (int ipoi = 0; ipoi < vPOI.size(); ipoi++)
		{
			fout << cvGetReal2D(vPOI[ipoi], 0, 0) << " " << cvGetReal2D(vPOI[ipoi], 1, 0) << " " << cvGetReal2D(vPOI[ipoi], 2, 0) << " ";
			fout << vf[ipoi] << " ";
			for (int ia = 0; ia < v_a[ipoi]->rows; ia++)
			{
				fout << cvGetReal2D(v_a[ipoi], ia, 0) << " " << cvGetReal2D(v_b[ipoi], ia, 0) << " " << cvGetReal2D(v_l[ipoi], ia, 0) << " "; 
			}
		}
		fout << endl;
	}
	fout.close();
}

void SavePOI_Match_Cov(string filename, vector<POI_Matches> vPOI, int nFrame, int nSegments_cov)
{
	PrintAlgorithm("Save Match Covariance Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPOIs: " << vPOI.size() << endl;
	fout << "NumSegments_Cov: " << nSegments_cov << endl;
	for (int iPOI = 0; iPOI < vPOI.size(); iPOI++)
	{
		fout << vPOI[iPOI].vx.size() << " ";
		for (int iFrame = 0; iFrame < vPOI[iPOI].vx.size(); iFrame++)
		{
			fout << vPOI[iPOI].vFrame[iFrame] << " " ;
			fout << vPOI[iPOI].vx[iFrame] << " " ;
			fout << vPOI[iPOI].vy[iFrame] << " " ;
			fout << vPOI[iPOI].vz[iFrame] << " " ;
			fout << vPOI[iPOI].v_f[iFrame] << " ";

			for (int ia = 0; ia < vPOI[iPOI].vv_a_cov[iFrame].size(); ia++)
			{
				fout << vPOI[iPOI].vv_a_cov[iFrame][ia] << " " << vPOI[iPOI].vv_b_cov[iFrame][ia] << " " << vPOI[iPOI].vv_l_cov[iFrame][ia] << " ";
			}
		}
		fout << endl;
	}
}

void LoadPOI_Match_refined(string filename, vector<POI_Matches> &vPOI, int &nseg)
{
	PrintAlgorithm("Load Match Refined Data");
	cout << "File name: " << filename << endl;
	ifstream fout;
	fout.open(filename.c_str(), ifstream::in);
	string dummy;
	int npoi;
	fout >> dummy >> npoi;
	int np = nseg*(nseg/2+1);
	for (int ipoi = 0; ipoi < npoi; ipoi++)
	{
		POI_Matches POI;
		int nx;
		fout >> nx;
		for (int iFrame = 0; iFrame < nx; iFrame++)
		{
			int frame;
			double x,y,z,f;
			fout >> frame >> x >> y >> z;
			POI.vFrame.push_back(frame);
			POI.vx.push_back(x);
			POI.vy.push_back(y);
			POI.vz.push_back(z);
		}
		vPOI.push_back(POI);
	}
	fout.close();
}

void SavePOI_Meanshift(string filename, vector<vector<CvMat *> > vvPOI, int nSegments_cov, vector<vector<CvMat *> > vv_a, vector<vector<CvMat*> > vv_b, vector<vector<CvMat *> > vv_l, vector<vector<double> > vvf, vector<int> vFrame,
					   vector<vector<vector<CvMat *> > > vvvMeanTraj)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumFrames: " << vvPOI.size() << endl;
	fout << "NumSegments_Cov: " << nSegments_cov << endl;
	for (int iFrame = 0; iFrame < vFrame.size(); iFrame++)
	{
		vector<CvMat *> vPOI = vvPOI[iFrame];
		vector<CvMat *> v_a = vv_a[iFrame];
		vector<CvMat *> v_b = vv_b[iFrame];
		vector<CvMat *> v_l = vv_l[iFrame];
		vector<double> vf = vvf[iFrame];
		fout << vFrame[iFrame] << " " << vPOI.size() << " ";
		for (int ipoi = 0; ipoi < vPOI.size(); ipoi++)
		{
			fout << cvGetReal2D(vPOI[ipoi], 0, 0) << " " << cvGetReal2D(vPOI[ipoi], 1, 0) << " " << cvGetReal2D(vPOI[ipoi], 2, 0) << " ";
			fout << vf[ipoi] << " ";
			for (int ia = 0; ia < v_a[ipoi]->rows; ia++)
			{
				fout << cvGetReal2D(v_a[ipoi], ia, 0) << " " << cvGetReal2D(v_b[ipoi], ia, 0) << " " << cvGetReal2D(v_l[ipoi], ia, 0) << " "; 
			}
		}
		fout << endl;
		vector<vector<CvMat *> > vvMeanTraj = vvvMeanTraj[iFrame];
		fout << vvMeanTraj.size() << endl;

		for (int imean = 0; imean < vvMeanTraj.size(); imean++)
		{
			fout << vvMeanTraj[imean].size() << " ";
			for (int imean_x = 0; imean_x < vvMeanTraj[imean].size(); imean_x++)
			{
				fout << cvGetReal2D(vvMeanTraj[imean][imean_x], 0, 0) << " " << cvGetReal2D(vvMeanTraj[imean][imean_x], 1, 0) << " " << cvGetReal2D(vvMeanTraj[imean][imean_x], 2, 0) << " "; 
			}
			fout << endl;
		}
	}
	fout.close();
}

void SaveRelativePoseData(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm, vector<Feature> &vFeature, vector<vector<int> > &vvVisibleID)
{
	PrintAlgorithm("Save Relative Pose Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPairs: " << vFrame1.size() << endl;

	for (int iFrame = 0; iFrame < vFrame1.size(); iFrame++)
	{
		fout << vFrame1[iFrame] << " " << vFrame2[iFrame] << endl;
		fout << cvGetReal2D(vm[iFrame], 0, 0) << " " << cvGetReal2D(vm[iFrame], 1, 0) << " " << cvGetReal2D(vm[iFrame], 2, 0) << endl; 

		fout << cvGetReal2D(vM[iFrame], 0, 0) << " " << cvGetReal2D(vM[iFrame], 0, 1) << " " << cvGetReal2D(vM[iFrame], 0, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 1, 0) << " " << cvGetReal2D(vM[iFrame], 1, 1) << " " << cvGetReal2D(vM[iFrame], 1, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 2, 0) << " " << cvGetReal2D(vM[iFrame], 2, 1) << " " << cvGetReal2D(vM[iFrame], 2, 2) << endl; 

		vector<int> vVisibleID = vvVisibleID[iFrame];
		vector<double> x1, y1, x2, y2;
		for (int iFeature = 0; iFeature < vVisibleID.size(); iFeature++)
		{
			vector<int>::iterator it1 = find(vFeature[vVisibleID[iFeature]].vFrame.begin(),vFeature[vVisibleID[iFeature]].vFrame.end(), vFrame1[iFrame]);
			vector<int>::iterator it2 = find(vFeature[vVisibleID[iFeature]].vFrame.begin(),vFeature[vVisibleID[iFeature]].vFrame.end(), vFrame2[iFrame]);

			//if ((it1 != vFeature[vVisibleID[iFeature]].vFrame.end()) && (it2 != vFeature[vVisibleID[iFeature]].vFrame.end()))
			{
				int idx = int(it1-vFeature[vVisibleID[iFeature]].vFrame.begin());
				x1.push_back(vFeature[vVisibleID[iFeature]].vx[idx]);
				y1.push_back(vFeature[vVisibleID[iFeature]].vy[idx]);
				idx = int(it2-vFeature[vVisibleID[iFeature]].vFrame.begin());
				x2.push_back(vFeature[vVisibleID[iFeature]].vx[idx]);
				y2.push_back(vFeature[vVisibleID[iFeature]].vy[idx]);
			}
		}

		fout << vVisibleID.size() << " ";
		for (int ix = 0; ix < x1.size(); ix++)
		{
			fout << x1[ix] << " " << y1[ix] << " " << x2[ix] << " " << y2[ix] << " ";
		}
		fout << endl;
	}
	fout.close();
}

void SaveRelativePoseDataAll(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm, vector<CvMat *> &vx1, vector<CvMat *> &vx2, int mode)
{
	PrintAlgorithm("Save Relative Pose Data");
	cout << "File name: " << filename << endl;
	ofstream fout;

	if (mode == FILESAVE_APPEND_MODE)
		fout.open(filename.c_str(), ios_base::app);
	else
	{
		fout.open(filename.c_str(), ios_base::out);
		fout.close();
		return;
	}
	fout << "NumPairs: " << vFrame1.size() << endl;

	for (int iFrame = 0; iFrame < vFrame1.size(); iFrame++)
	{
		fout << vFrame1[iFrame] << " " << vFrame2[iFrame] << endl;
		fout << cvGetReal2D(vm[iFrame], 0, 0) << " " << cvGetReal2D(vm[iFrame], 1, 0) << " " << cvGetReal2D(vm[iFrame], 2, 0) << endl; 

		fout << cvGetReal2D(vM[iFrame], 0, 0) << " " << cvGetReal2D(vM[iFrame], 0, 1) << " " << cvGetReal2D(vM[iFrame], 0, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 1, 0) << " " << cvGetReal2D(vM[iFrame], 1, 1) << " " << cvGetReal2D(vM[iFrame], 1, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 2, 0) << " " << cvGetReal2D(vM[iFrame], 2, 1) << " " << cvGetReal2D(vM[iFrame], 2, 2) << endl; 

		if (vx1[iFrame]->rows == 1)
		{
			fout << 0 << endl;
			continue;
		}
		fout << vx1[iFrame]->rows << " ";

		for (int ix = 0; ix < vx1[iFrame]->rows; ix++)
		{
			fout << cvGetReal2D(vx1[iFrame], ix, 0) << " " << cvGetReal2D(vx1[iFrame], ix, 1) << " " << cvGetReal2D(vx2[iFrame], ix, 0) << " " << cvGetReal2D(vx2[iFrame], ix, 1) << " ";
		}
		fout << endl;
	}
	fout.close();
}

void SaveRelativePoseDataAll(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm, int mode)
{
	PrintAlgorithm("Save Relative Pose Data");
	cout << "File name: " << filename << endl;
	ofstream fout;

	if (mode == FILESAVE_APPEND_MODE)
		fout.open(filename.c_str(), ios_base::app);
	else
	{
		fout.open(filename.c_str(), ios_base::out);
		fout.close();
		return;
	}
	fout << "NumPairs: " << vFrame1.size() << endl;

	for (int iFrame = 0; iFrame < vFrame1.size(); iFrame++)
	{
		fout << vFrame1[iFrame] << " " << vFrame2[iFrame] << endl;
		fout << cvGetReal2D(vm[iFrame], 0, 0) << " " << cvGetReal2D(vm[iFrame], 1, 0) << " " << cvGetReal2D(vm[iFrame], 2, 0) << endl; 

		fout << cvGetReal2D(vM[iFrame], 0, 0) << " " << cvGetReal2D(vM[iFrame], 0, 1) << " " << cvGetReal2D(vM[iFrame], 0, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 1, 0) << " " << cvGetReal2D(vM[iFrame], 1, 1) << " " << cvGetReal2D(vM[iFrame], 1, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 2, 0) << " " << cvGetReal2D(vM[iFrame], 2, 1) << " " << cvGetReal2D(vM[iFrame], 2, 2) << endl; 
	}
	fout.close();
}

void SaveRelativePoseData(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm, vector<CvMat *> &vx1, vector<CvMat *> &vx2)
{
	PrintAlgorithm("Save Relative Pose Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPairs: " << vFrame1.size() << endl;

	for (int iFrame = 0; iFrame < vFrame1.size(); iFrame++)
	{
		fout << vFrame1[iFrame] << " " << vFrame2[iFrame] << endl;
		fout << cvGetReal2D(vm[iFrame], 0, 0) << " " << cvGetReal2D(vm[iFrame], 1, 0) << " " << cvGetReal2D(vm[iFrame], 2, 0) << endl; 

		fout << cvGetReal2D(vM[iFrame], 0, 0) << " " << cvGetReal2D(vM[iFrame], 0, 1) << " " << cvGetReal2D(vM[iFrame], 0, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 1, 0) << " " << cvGetReal2D(vM[iFrame], 1, 1) << " " << cvGetReal2D(vM[iFrame], 1, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 2, 0) << " " << cvGetReal2D(vM[iFrame], 2, 1) << " " << cvGetReal2D(vM[iFrame], 2, 2) << endl; 

		fout << vx1[iFrame]->rows << " ";

		for (int ix = 0; ix < vx1[iFrame]->rows; ix++)
		{
			fout << cvGetReal2D(vx1[iFrame], ix, 0) << " " << cvGetReal2D(vx1[iFrame], ix, 1) << " " << cvGetReal2D(vx2[iFrame], ix, 0) << " " << cvGetReal2D(vx2[iFrame], ix, 1) << " ";
		}
		fout << endl;
	}
	fout.close();
}

void SaveRelativePoseData(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm)
{
	PrintAlgorithm("Save Relative Pose Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPairs: " << vFrame1.size() << endl;

	for (int iFrame = 0; iFrame < vFrame1.size(); iFrame++)
	{
		fout << vFrame1[iFrame] << " " << vFrame2[iFrame] << endl;
		fout << cvGetReal2D(vm[iFrame], 0, 0) << " " << cvGetReal2D(vm[iFrame], 1, 0) << " " << cvGetReal2D(vm[iFrame], 2, 0) << endl; 

		fout << cvGetReal2D(vM[iFrame], 0, 0) << " " << cvGetReal2D(vM[iFrame], 0, 1) << " " << cvGetReal2D(vM[iFrame], 0, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 1, 0) << " " << cvGetReal2D(vM[iFrame], 1, 1) << " " << cvGetReal2D(vM[iFrame], 1, 2) << endl; 
		fout << cvGetReal2D(vM[iFrame], 2, 0) << " " << cvGetReal2D(vM[iFrame], 2, 1) << " " << cvGetReal2D(vM[iFrame], 2, 2) << endl; 
	}
	fout.close();
}

void LoadRelativePoseData(string filename, vector<int> &vFrame1, vector<int> &vFrame2, vector<CvMat*> &vM, vector<CvMat *> &vm, vector<CvMat *> &vx1, vector<CvMat *> &vx2)
{
	PrintAlgorithm("Load Relative Pose Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a;
	int nFrames;
	file >> nFrames;

	for (int iFrame = 0; iFrame < nFrames; iFrame++)
	{
		int frame1, frame2;
		CvMat *M = cvCreateMat(3,3,CV_32FC1);
		CvMat *m = cvCreateMat(3,1,CV_32FC1);
		
		file >> frame1 >> frame2;
		vFrame1.push_back(frame1);
		vFrame2.push_back(frame2);

		double m11, m21, m31;
		double M11, M12, M13, M21, M22, M23, M31, M32, M33;
		file >> m11 >> m21 >> m31;
		file >> M11 >> M12 >> M13;
		file >> M21 >> M22 >> M23;
		file >> M31 >> M32 >> M33;

		cvSetReal2D(m, 0, 0, m11);		
		cvSetReal2D(m, 1, 0, m21);		
		cvSetReal2D(m, 2, 0, m31);	

		cvSetReal2D(M, 0, 0, M11);			cvSetReal2D(M, 0, 1, M12);			cvSetReal2D(M, 0, 2, M13);
		cvSetReal2D(M, 1, 0, M21);			cvSetReal2D(M, 1, 1, M22);			cvSetReal2D(M, 1, 2, M23);
		cvSetReal2D(M, 2, 0, M31);			cvSetReal2D(M, 2, 1, M32);			cvSetReal2D(M, 2, 2, M33);

		int nPoints;
		file >> nPoints;
		CvMat *x1 = cvCreateMat(nPoints, 2, CV_32FC1);
		CvMat *x2 = cvCreateMat(nPoints, 2, CV_32FC1);
		for (int iPoint = 0; iPoint < nPoints; iPoint++)
		{
			double mx1, my1, mx2, my2;
			file >> mx1 >> my1 >> mx2 >> my2;
			cvSetReal2D(x1, iPoint, 0, mx1);
			cvSetReal2D(x1, iPoint, 1, my1);

			cvSetReal2D(x2, iPoint, 0, mx2);
			cvSetReal2D(x2, iPoint, 1, my2);
		}
		vx1.push_back(x1);
		vx2.push_back(x2);
		vM.push_back(M);
		vm.push_back(m);
	}
	file.close();
}

void LoadQueryFileData(string filename, Camera &camera)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	file >> a >> a;
	int nFrame;
	file >> a >> nFrame;
	file >> a >> nFrame;

	for (int i = 0; i < nFrame; i++)
	{
		file >> a;
		camera.vFileName.push_back(a);
	}
	file.close();
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

void SaveCameraData_DistortionK(string filename, vector<Camera> vCamera, vector<CvMat*> cK, vector<double> vk1, vector<int> vUsedFrame, int max_nFrames)
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
	fout << "NumP " << cK.size() << endl;

	for (int i = 0; i < cK.size(); i++)
	{
		int frame = vUsedFrame[i]%max_nFrames;
		int cam = (int) vUsedFrame[i]/max_nFrames;
		vector<int>:: const_iterator it = find(vCamera[cam].vTakenFrame.begin(), vCamera[cam].vTakenFrame.end(), frame);
		if (it == vCamera[cam].vTakenFrame.end())
			return;
		int iTakenFrame = (int) (it - vCamera[cam].vTakenFrame.begin());

		fout << vCamera[cam].id << " " << iTakenFrame << endl;
		fout << vk1[i] << endl;
		fout << cvGetReal2D(cK[i], 0, 0) << " ";
		fout << cvGetReal2D(cK[i], 1, 1) << " ";
		fout << cvGetReal2D(cK[i], 0, 2) << " ";
		fout << cvGetReal2D(cK[i], 1, 2) << endl;
	}	

	fout.close();
}

void SaveCameraInfoData(string filename, Camera camera)
{
	PrintAlgorithm("Save Camera Info Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams 1" << endl;
	fout << "NumFrames " << camera.vC.size() << endl;
	fout << "NumP " << camera.vC.size() << endl;

	for (int i = 0; i < camera.vC.size(); i++)
	{
		fout << "0 " << camera.vTakenFrame[i] << endl;
		fout << cvGetReal2D(camera.vC[i], 0, 0) << " " << cvGetReal2D(camera.vC[i], 1, 0) << " " << cvGetReal2D(camera.vC[i], 2, 0) << endl;
		fout << cvGetReal2D(camera.vR[i], 0, 0) << " " << cvGetReal2D(camera.vR[i], 0, 1) << " " << cvGetReal2D(camera.vR[i], 0, 2) << endl;
		fout << cvGetReal2D(camera.vR[i], 1, 0) << " " << cvGetReal2D(camera.vR[i], 1, 1) << " " << cvGetReal2D(camera.vR[i], 1, 2) << endl;
		fout << cvGetReal2D(camera.vR[i], 2, 0) << " " << cvGetReal2D(camera.vR[i], 2, 1) << " " << cvGetReal2D(camera.vR[i], 2, 2) << endl;

		fout << camera.vImageName[i] << " " << camera.vk1[i] << endl;
		fout << cvGetReal2D(camera.vK[i], 0, 0) << " " << cvGetReal2D(camera.vK[i], 0, 1) << " " << cvGetReal2D(camera.vK[i], 0, 2) << endl;
		fout << cvGetReal2D(camera.vK[i], 1, 0) << " " << cvGetReal2D(camera.vK[i], 1, 1) << " " << cvGetReal2D(camera.vK[i], 1, 2) << endl;
		fout << cvGetReal2D(camera.vK[i], 2, 0) << " " << cvGetReal2D(camera.vK[i], 2, 1) << " " << cvGetReal2D(camera.vK[i], 2, 2) << endl;
	}	

	fout.close();
}

void SaveCameraData_KRT(string filename, Camera camera)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams 1" << endl;
	fout << "NumFrames " << camera.vC.size() << endl;
	fout << "NumP " << camera.vC.size() << endl;

	for (int i = 0; i < camera.vC.size(); i++)
	{
		fout << "0 " << camera.vTakenFrame[i] << endl;
		fout << cvGetReal2D(camera.vC[i], 0, 0) << " " << cvGetReal2D(camera.vC[i], 1, 0) << " " << cvGetReal2D(camera.vC[i], 2, 0) << endl;
		fout << cvGetReal2D(camera.vR[i], 0, 0) << " " << cvGetReal2D(camera.vR[i], 0, 1) << " " << cvGetReal2D(camera.vR[i], 0, 2) << endl;
		fout << cvGetReal2D(camera.vR[i], 1, 0) << " " << cvGetReal2D(camera.vR[i], 1, 1) << " " << cvGetReal2D(camera.vR[i], 1, 2) << endl;
		fout << cvGetReal2D(camera.vR[i], 2, 0) << " " << cvGetReal2D(camera.vR[i], 2, 1) << " " << cvGetReal2D(camera.vR[i], 2, 2) << endl;
	}	

	fout.close();
}

void SaveQueryFileData(string filename, Camera camera)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams 1" << endl;
	fout << "NumFrames " << camera.vC.size() << endl;
	fout << "NumP " << camera.vC.size() << endl;

	for (int i = 0; i < camera.vFileName.size(); i++)
	{
		fout << camera.vFileName[i] << endl;
	}	

	fout.close();
}

void SaveCameraIntrisicData_CP(string filename, Camera camera)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams 1" << endl;
	fout << "NumFrames " << camera.vC.size() << endl;
	fout << "NumP " << camera.vC.size() << endl;

	for (int i = 0; i < camera.vC.size(); i++)
	{
		fout << "0 " << i << endl;
		fout << cvGetReal2D(camera.vK[i], 0, 0) << " " << cvGetReal2D(camera.vK[i], 0, 1) << " " << cvGetReal2D(camera.vK[i], 0, 2) << endl;
		fout << cvGetReal2D(camera.vK[i], 1, 0) << " " << cvGetReal2D(camera.vK[i], 1, 1) << " " << cvGetReal2D(camera.vK[i], 1, 2) << endl;
		fout << cvGetReal2D(camera.vK[i], 2, 0) << " " << cvGetReal2D(camera.vK[i], 2, 1) << " " << cvGetReal2D(camera.vK[i], 2, 2) << endl;
		fout << camera.vk1[i] << " " << camera.vk2[i] << endl;
	}	

	fout.close();
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

void LoadCameraIntrisicData_CP(string filename, Camera &camera)
{
	PrintAlgorithm("Load Camera Intrinsic Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int nframes;
	file >> a >> a;
	file >> a >> nframes;
	file >> a >> nframes;

	for (int i = 0; i < nframes; i++)
	{
		int frame;
		double k11, k12, k13;
		double k21, k22, k23;
		double k31, k32, k33;
		double k1, k2;
		file >> a >> frame;
		file >> k11 >> k12 >> k13;
		file >> k21 >> k22 >> k23;
		file >> k31 >> k32 >> k33;
		file >> k1 >> k2;

		CvMat *K = cvCreateMat(3,3,CV_32FC1);
		cvSetReal2D(K, 0, 0, k11);	cvSetReal2D(K, 0, 1, k12);	cvSetReal2D(K, 0, 2, k13);
		cvSetReal2D(K, 1, 0, k21);	cvSetReal2D(K, 1, 1, k22);	cvSetReal2D(K, 1, 2, k23);
		cvSetReal2D(K, 2, 0, k31);	cvSetReal2D(K, 2, 1, k32);	cvSetReal2D(K, 2, 2, k33);
		camera.vK.push_back(K);
		camera.vk1.push_back(k1);
		camera.vk2.push_back(k2);
	}
	file.close();
}

void SaveCameraData_KRT_AD(string filename, Camera camera, vector<vector<int> > vvPointIdx)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams 1" << endl;
	fout << "NumFrames " << camera.vC.size() << endl;
	fout << "NumP " << camera.vC.size() << endl;

	for (int i = 0; i < camera.vC.size(); i++)
	{
		fout << "0 " << i << endl;
		fout << cvGetReal2D(camera.vC[i], 0, 0) << " " << cvGetReal2D(camera.vC[i], 1, 0) << " " << cvGetReal2D(camera.vC[i], 2, 0) << endl;
		fout << cvGetReal2D(camera.vR[i], 0, 0) << " " << cvGetReal2D(camera.vR[i], 0, 1) << " " << cvGetReal2D(camera.vR[i], 0, 2) << endl;
		fout << cvGetReal2D(camera.vR[i], 1, 0) << " " << cvGetReal2D(camera.vR[i], 1, 1) << " " << cvGetReal2D(camera.vR[i], 1, 2) << endl;
		fout << cvGetReal2D(camera.vR[i], 2, 0) << " " << cvGetReal2D(camera.vR[i], 2, 1) << " " << cvGetReal2D(camera.vR[i], 2, 2) << endl;

		fout << vvPointIdx[i].size() << " ";
		for (int iPoint = 0; iPoint < vvPointIdx[i].size(); iPoint++)
		{
			fout << vvPointIdx[i][iPoint] << " ";
		}
		fout << endl;
	}	

	fout.close();
}

void SaveCameraData_KRT_AD(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames,
	vector<vector<int> > vvPointIndex)
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

		fout << vvPointIndex[i].size() << " ";
		for (int iPoint = 0; iPoint < vvPointIndex[i].size(); iPoint++)
		{
			fout << vvPointIndex[i][iPoint] << " ";
		}
		fout << endl;

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

void SaveStructureData(string filename, vector<double> vX, vector<double> vY, vector<double> vZ)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << vX.size() << endl;
	for (int i = 0; i < vX.size(); i++)
	{
		fout << i << " 255 255 255 " << vX[i] << " " << vY[i] << " " << vZ[i] << endl;
	}	
	fout.close();
}

void SaveFaceData(string filename, vector<vector<Face> > vvFace, vector<vector<double> > vvFace_x, vector<vector<double> > vvFace_y, vector<vector<double> > vvFace_z)
{
	PrintAlgorithm("Save Face Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumFaces: " << vvFace.size() << endl;

	for (int iFace = 0; iFace < vvFace.size(); iFace++)
	{
		fout << vvFace[iFace].size() << " " << vvFace_x[iFace].size() << " ";

		double v1, v2, v3, u1, u2, u3;
		int idx1, idx2, idx3;

		idx1 = 0;	idx2 = 31;	idx3 = 9;
		u1 = vvFace_x[iFace][idx2]-vvFace_x[iFace][idx1];
		u2 = vvFace_y[iFace][idx2]-vvFace_y[iFace][idx1];
		u3 = vvFace_z[iFace][idx2]-vvFace_z[iFace][idx1];

		v1 = vvFace_x[iFace][idx3]-vvFace_x[iFace][idx1];
		v2 = vvFace_y[iFace][idx3]-vvFace_y[iFace][idx1];
		v3 = vvFace_z[iFace][idx3]-vvFace_z[iFace][idx1];

		double n1x, n1y, n1z;
		GetNormalVector(u1, u2, u3, v1, v2, v3, n1x, n1y, n1z);

		idx1 = 9;	idx2 = 0;	idx3 = 37;
		u1 = vvFace_x[iFace][idx2]-vvFace_x[iFace][idx1];
		u2 = vvFace_y[iFace][idx2]-vvFace_y[iFace][idx1];
		u3 = vvFace_z[iFace][idx2]-vvFace_z[iFace][idx1];

		v1 = vvFace_x[iFace][idx3]-vvFace_x[iFace][idx1];
		v2 = vvFace_y[iFace][idx3]-vvFace_y[iFace][idx1];
		v3 = vvFace_z[iFace][idx3]-vvFace_z[iFace][idx1];

		double n2x, n2y, n2z;
		GetNormalVector(u1, u2, u3, v1, v2, v3, n2x, n2y, n2z);

		idx1 = 37;	idx2 = 9;	idx3 = 31;
		u1 = vvFace_x[iFace][idx2]-vvFace_x[iFace][idx1];
		u2 = vvFace_y[iFace][idx2]-vvFace_y[iFace][idx1];
		u3 = vvFace_z[iFace][idx2]-vvFace_z[iFace][idx1];

		v1 = vvFace_x[iFace][idx3]-vvFace_x[iFace][idx1];
		v2 = vvFace_y[iFace][idx3]-vvFace_y[iFace][idx1];
		v3 = vvFace_z[iFace][idx3]-vvFace_z[iFace][idx1];

		double n3x, n3y, n3z;
		GetNormalVector(u1, u2, u3, v1, v2, v3, n3x, n3y, n3z);

		idx1 = 31;	idx2 = 37;	idx3 = 0;
		u1 = vvFace_x[iFace][idx2]-vvFace_x[iFace][idx1];
		u2 = vvFace_y[iFace][idx2]-vvFace_y[iFace][idx1];
		u3 = vvFace_z[iFace][idx2]-vvFace_z[iFace][idx1];

		v1 = vvFace_x[iFace][idx3]-vvFace_x[iFace][idx1];
		v2 = vvFace_y[iFace][idx3]-vvFace_y[iFace][idx1];
		v3 = vvFace_z[iFace][idx3]-vvFace_z[iFace][idx1];

		double n4x, n4y, n4z;
		GetNormalVector(u1, u2, u3, v1, v2, v3, n4x, n4y, n4z);

		double face_normal_x = (n1x+n2x+n3x+n4x)/4;
		double face_normal_y = (n1y+n2y+n3y+n4y)/4;
		double face_normal_z = (n1z+n2z+n3z+n4z)/4;

		double norm_normal = sqrt(face_normal_x*face_normal_x+face_normal_y*face_normal_y+face_normal_z*face_normal_z);
		face_normal_x /= norm_normal;
		face_normal_y /= norm_normal;
		face_normal_z /= norm_normal;

		fout << vvFace_x[iFace][10] << " " << vvFace_y[iFace][10] << " " << vvFace_z[iFace][10] << " ";
		fout << face_normal_x << " " << face_normal_y << " " << face_normal_z << " ";

		for (int i = 0; i < vvFace_x[iFace].size(); i++)
		{
			fout << vvFace_x[iFace][i] << " " << vvFace_y[iFace][i] << " " << vvFace_z[iFace][i] << " ";
		}

		for (int i = 0; i < vvFace[iFace].size(); i++)
		{
			fout << vvFace[iFace][i].panel << " " << vvFace[iFace][i].camera << " ";
		}
		fout << endl;
	}
	fout.close();
}

void SaveFaceData_Direction(string filename, vector<vector<Face> > vvFace, vector<vector<double> > vvFace_x, vector<vector<double> > vvFace_y, vector<vector<double> > vvFace_z)
{
	PrintAlgorithm("Save Face Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumFaces: " << vvFace.size() << endl;


	for (int iFace = 0; iFace < vvFace.size(); iFace++)
	{
		fout << vvFace[iFace].size() << " " << vvFace_x[iFace].size() << " ";

		int idx = 10;

		CvMat *A = cvCreateMat(48, 2, CV_32FC1);
		CvMat *b = cvCreateMat(48, 1, CV_32FC1);

		int id=0;
		for (int i = 0; i < vvFace_x[iFace].size(); i++)
		{
			if (i == idx)
				continue;		
			double dx = vvFace_x[iFace][i]-vvFace_x[iFace][idx];
			double dy = vvFace_y[iFace][i]-vvFace_y[iFace][idx];
			double dz = vvFace_z[iFace][i]-vvFace_z[iFace][idx];

			double norm = sqrt(dx*dx+dy*dy+dz*dz);
			dx /= norm;
			dy /= norm;
			dz /= norm;
			cvSetReal2D(A, id, 0, dx);
			cvSetReal2D(A, id, 1, dy);
			cvSetReal2D(b, id, 0, -dz);
			id++;
		}
		CvMat *v = cvCreateMat(2,1,CV_32FC1);
		cvSolve(A, b, v);

		

		double vx = cvGetReal2D(v, 0, 0);
		double vy = cvGetReal2D(v, 1, 0);
		double vz = 1;

		double norm_v = sqrt(vx*vx+vy*vy+vz*vz);
		vx /= norm_v;
		vy /= norm_v;
		vz /= norm_v;

		cvReleaseMat(&A);
		cvReleaseMat(&b);
		cvReleaseMat(&v);

		double v1, v2, v3, u1, u2, u3;
		int idx1, idx2, idx3;

		idx1 = 0;	idx2 = 31;	idx3 = 9;
		u1 = vvFace_x[iFace][idx2]-vvFace_x[iFace][idx1];
		u2 = vvFace_y[iFace][idx2]-vvFace_y[iFace][idx1];
		u3 = vvFace_z[iFace][idx2]-vvFace_z[iFace][idx1];

		v1 = vvFace_x[iFace][idx3]-vvFace_x[iFace][idx1];
		v2 = vvFace_y[iFace][idx3]-vvFace_y[iFace][idx1];
		v3 = vvFace_z[iFace][idx3]-vvFace_z[iFace][idx1];

		double n1x, n1y, n1z;
		GetNormalVector(u1, u2, u3, v1, v2, v3, n1x, n1y, n1z);

		idx1 = 9;	idx2 = 0;	idx3 = 37;
		u1 = vvFace_x[iFace][idx2]-vvFace_x[iFace][idx1];
		u2 = vvFace_y[iFace][idx2]-vvFace_y[iFace][idx1];
		u3 = vvFace_z[iFace][idx2]-vvFace_z[iFace][idx1];

		v1 = vvFace_x[iFace][idx3]-vvFace_x[iFace][idx1];
		v2 = vvFace_y[iFace][idx3]-vvFace_y[iFace][idx1];
		v3 = vvFace_z[iFace][idx3]-vvFace_z[iFace][idx1];

		double n2x, n2y, n2z;
		GetNormalVector(u1, u2, u3, v1, v2, v3, n2x, n2y, n2z);

		idx1 = 37;	idx2 = 9;	idx3 = 31;
		u1 = vvFace_x[iFace][idx2]-vvFace_x[iFace][idx1];
		u2 = vvFace_y[iFace][idx2]-vvFace_y[iFace][idx1];
		u3 = vvFace_z[iFace][idx2]-vvFace_z[iFace][idx1];

		v1 = vvFace_x[iFace][idx3]-vvFace_x[iFace][idx1];
		v2 = vvFace_y[iFace][idx3]-vvFace_y[iFace][idx1];
		v3 = vvFace_z[iFace][idx3]-vvFace_z[iFace][idx1];

		double n3x, n3y, n3z;
		GetNormalVector(u1, u2, u3, v1, v2, v3, n3x, n3y, n3z);

		idx1 = 31;	idx2 = 37;	idx3 = 0;
		u1 = vvFace_x[iFace][idx2]-vvFace_x[iFace][idx1];
		u2 = vvFace_y[iFace][idx2]-vvFace_y[iFace][idx1];
		u3 = vvFace_z[iFace][idx2]-vvFace_z[iFace][idx1];

		v1 = vvFace_x[iFace][idx3]-vvFace_x[iFace][idx1];
		v2 = vvFace_y[iFace][idx3]-vvFace_y[iFace][idx1];
		v3 = vvFace_z[iFace][idx3]-vvFace_z[iFace][idx1];

		double n4x, n4y, n4z;
		GetNormalVector(u1, u2, u3, v1, v2, v3, n4x, n4y, n4z);

		double face_normal_x = (n1x+n2x+n3x+n4x)/4;
		double face_normal_y = (n1y+n2y+n3y+n4y)/4;
		double face_normal_z = (n1z+n2z+n3z+n4z)/4;

		double norm_normal = sqrt(face_normal_x*face_normal_x+face_normal_y*face_normal_y+face_normal_z*face_normal_z);
		face_normal_x /= norm_normal;
		face_normal_y /= norm_normal;
		face_normal_z /= norm_normal;

		double dot = face_normal_x*vx+face_normal_y*vy+face_normal_z*vz;
		if (dot < 0)
		{
			vx *= -1;
			vy *= -1;
			vz *= -1;
		}


		fout << vvFace_x[iFace][10] << " " << vvFace_y[iFace][10] << " " << vvFace_z[iFace][10] << " ";
		fout << vx << " " << vy << " " << vz << " ";

		for (int i = 0; i < vvFace_x[iFace].size(); i++)
		{
			fout << vvFace_x[iFace][i] << " " << vvFace_y[iFace][i] << " " << vvFace_z[iFace][i] << " ";
		}

		for (int i = 0; i < vvFace[iFace].size(); i++)
		{
			fout << vvFace[iFace][i].panel << " " << vvFace[iFace][i].camera << " ";
		}
		fout << endl;
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

void SaveStructureData_SO(string filename, vector<double> vX, vector<double> vY, vector<double> vZ, 
							vector<double> vX1, vector<double> vY1, vector<double> vZ1,
							vector<int> vR, vector<int> vG, vector<int> vB, vector<int> vID)
{
	PrintAlgorithm("Save Structure SO Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << vID.size() << endl;
	for (int i = 0; i < vID.size(); i++)
	{
		fout << vID[i] << " " << vR[i] << " " << vG[i] << " " << vB[i] << " ";
		fout << vX[i] << " " << vY[i] << " " << vZ[i] << " ";
		fout << vX1[i] << " " << vY1[i] << " " << vZ1[i] << endl;
	}	
	fout.close();
}

void SaveStructureData_RGB(string filename, vector<double> vX, vector<double> vY, vector<double> vZ, vector<int> ID, vector<Feature> vFeature)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << ID.size() << endl;
	for (int i = 0; i < ID.size(); i++)
	{
		fout << ID[i] << " ";
		fout << vFeature[ID[i]].r << " " << vFeature[ID[i]].g << " " << vFeature[ID[i]].b << " "; 
		fout << vX[i] << " " << vY[i] << " " << vZ[i] << endl;
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

void SaveStructureData_RGB_fast_3More(string filename, CvMat *X, vector<Feature> &vFeature)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	int count = 0;
	for (int i = 0; i < vFeature.size(); i++)
	{
		if ((vFeature[i].isRegistered) && (vFeature[i].nProj > 2))
			count++;
	}
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumPoints " << count << endl;
	for (int i = 0; i < vFeature.size(); i++)
	{
		if ((vFeature[i].isRegistered) && (vFeature[i].nProj > 2))
		{
			fout << vFeature[i].id << " ";
			fout << vFeature[i].r << " " << vFeature[i].g << " " << vFeature[i].b << " "; 
			fout << cvGetReal2D(X, i, 0) << " " << cvGetReal2D(X, i, 1) << " " << cvGetReal2D(X, i, 2) << endl;
		}
	}	
	fout.close();
}


void SaveStructureData_RGB_fast_tripple(string filename, CvMat *X, vector<Feature> &vFeature)
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
		if (vFeature[i].vVisibleFrame.size() >= 3)
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

void LoadPOIData(string filename, vector<POI> &vPOI)
{
	PrintAlgorithm("Load POI Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFeatures;
	fin >> a >> nFeatures;
	int nBasis;
	fin >> a >> nBasis;
	int nFrames;
	fin >> a >> nFrames;
	for (int i = 0; i < nFeatures; i++)
	{
		int id;
		POI poi;
		vector<double> vx, vy, vz;
		fin >> id;
		poi.id = id;
		for (int iFrame = 0; iFrame < nFrames; iFrame++)
		{
			double x, y, z;
			fin >> x >> y >> z;
			poi.vx.push_back(x);
			poi.vy.push_back(y);
			poi.vz.push_back(z);
		}

		vPOI.push_back(poi);
	}	
	fin.close();
}

//void LoadPOIData_MeanShift(string filename, vector<POI_MeanShift> &vPOI)
//{
//	PrintAlgorithm("Load POI Data");
//	cout << "File name: " << filename << endl;
//	ifstream fin;
//	fin.open(filename.c_str(), ifstream::in);
//	string a;
//	int nFrames;
//	fin >> a >> nFrames;
//	fin >> a >> nSegments_cov;
//	for (int i = 0; i < nFrames; i++)
//	{
//		int frame;
//		int npoi;
//
//		POI_MeanShift poi;
//		fin >> frame >> npoi;
//		if (npoi == 0)
//			continue;
//		for (int ip = 0; ip < npoi; ip++)
//		{
//			double x, y, z;
//			fin >> x >> y >> z;
//			poi.vx.push_back(x);
//			poi.vy.push_back(y);
//			poi.vz.push_back(z);
//			poi.frame = frame;
//
//			//double f;
//			//fin >> f;
//			//poi.vf.push_back(f);
//
//			//int np = nSegments_cov*(nSegments_cov/2+1);
//			//CvMat *am = cvCreateMat(np, 1, CV_32FC1);
//			//CvMat *bm = cvCreateMat(np, 1, CV_32FC1);
//			//CvMat *lm = cvCreateMat(np, 1, CV_32FC1);
//			//for (int ip = 0; ip < np; ip++)
//			//{
//			//	double a, b, l;
//			//	fin >> a >> b >> l;
//			//	cvSetReal2D(am, ip, 0, a);
//			//	cvSetReal2D(bm, ip, 0, b);
//			//	cvSetReal2D(lm, ip, 0, l);
//			//}
//			//poi.v_a.push_back(am);
//			//poi.v_b.push_back(bm);
//			//poi.v_l.push_back(lm);
//		}
//		vPOI.push_back(poi);
//	}
//	fin.close();
//}

void LoadPOIData_MeanShift(string filename, vector<POI_MeanShift> &vPOI, int &nSegments_cov)
{
	PrintAlgorithm("Load POI Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFrames;
	fin >> a >> nFrames;
	fin >> a >> nSegments_cov;
	for (int i = 0; i < nFrames; i++)
	{
		int frame;
		int npoi;
		
		POI_MeanShift poi;
		fin >> frame >> npoi;
		if (npoi == 0)
			continue;
		for (int ip = 0; ip < npoi; ip++)
		{
			double x, y, z;
			fin >> x >> y >> z;
			poi.vx.push_back(x);
			poi.vy.push_back(y);
			poi.vz.push_back(z);
			poi.frame = frame;
			
			double f;
			fin >> f;
			poi.vf.push_back(f);
			
			int np = nSegments_cov*(nSegments_cov/2+1);
			CvMat *am = cvCreateMat(np, 1, CV_32FC1);
			CvMat *bm = cvCreateMat(np, 1, CV_32FC1);
			CvMat *lm = cvCreateMat(np, 1, CV_32FC1);
			for (int ip = 0; ip < np; ip++)
			{
				double a, b, l;
				fin >> a >> b >> l;
				cvSetReal2D(am, ip, 0, a);
				cvSetReal2D(bm, ip, 0, b);
				cvSetReal2D(lm, ip, 0, l);
			}
			poi.v_a.push_back(am);
			poi.v_b.push_back(bm);
			poi.v_l.push_back(lm);
		}
		vPOI.push_back(poi);
	}
	fin.close();
}

void LoadPOIData_MeanShift(string filename, vector<POI_MeanShift> &vPOI, int &nSegments_cov, vector<vector<vector<CvMat *> > > &vvvMeanTraj)
{
	PrintAlgorithm("Load POI Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFrames;
	fin >> a >> nFrames;
	fin >> a >> nSegments_cov;
	for (int i = 0; i < nFrames; i++)
	{
		int frame;
		int npoi;

		POI_MeanShift poi;
		fin >> frame >> npoi;
		if (npoi == 0)
			continue;
		for (int ip = 0; ip < npoi; ip++)
		{
			double x, y, z;
			fin >> x >> y >> z;
			poi.vx.push_back(x);
			poi.vy.push_back(y);
			poi.vz.push_back(z);
			poi.frame = frame;

			double f;
			fin >> f;
			poi.vf.push_back(f);

			int np = nSegments_cov*(nSegments_cov/2+1);
			CvMat *am = cvCreateMat(np, 1, CV_32FC1);
			CvMat *bm = cvCreateMat(np, 1, CV_32FC1);
			CvMat *lm = cvCreateMat(np, 1, CV_32FC1);
			for (int ip = 0; ip < np; ip++)
			{
				double a, b, l;
				fin >> a >> b >> l;
				cvSetReal2D(am, ip, 0, a);
				cvSetReal2D(bm, ip, 0, b);
				cvSetReal2D(lm, ip, 0, l);
			}
			poi.v_a.push_back(am);
			poi.v_b.push_back(bm);
			poi.v_l.push_back(lm);
		}
		vPOI.push_back(poi);

		int nTrajs;
		fin >> nTrajs;
		vector<vector<CvMat *> > vvMeanTraj;
		for (int ii = 0; ii < nTrajs; ii++)
		{
			int nSteps;
			fin >> nSteps;
			vector<CvMat *> vMeanTraj;
			for (int iStep = 0; iStep < nSteps; iStep++)
			{
				CvMat *mm = cvCreateMat(3,1,CV_32FC1);
				double xm,ym,zm;
				fin >> xm >> ym >> zm;
				cvSetReal2D(mm, 0, 0, xm);
				cvSetReal2D(mm, 1, 0, ym);
				cvSetReal2D(mm, 2, 0, zm);
				vMeanTraj.push_back(mm);
			}
			vvMeanTraj.push_back(vMeanTraj);
		}
		vvvMeanTraj.push_back(vvMeanTraj);
	}
	fin.close();
}

void LoadPOIData_MeanShift_GT(string filename, vector<POI_MeanShift> &vPOI)
{
	PrintAlgorithm("Load POI GT Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	string a;
	int nFrames;
	fin >> a >> nFrames;
	for (int i = 0; i < nFrames; i++)
	{
		int frame;
		int npoi;

		POI_MeanShift poi;
		fin >> frame >> npoi;
		if (npoi == 0)
			continue;
		for (int ip = 0; ip < npoi; ip++)
		{
			double x, y, z;
			fin >> x >> y >> z;
			poi.vx.push_back(x);
			poi.vy.push_back(y);
			poi.vz.push_back(z);
			poi.frame = frame;
		}
		vPOI.push_back(poi);
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

void LoadCorrespondence2D3DData(string filename, vector<vector<Correspondence2D3D> > &vvCorr, vector<int> &vFrame)
{
	PrintAlgorithm("Load FileList Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	while(!fin.eof())
	{

		int frame;
		fin >> frame;

		//if (frame > 300)
		//	break;

		vFrame.push_back(frame);
		int nPoint;
		fin >> nPoint;

		vector<Correspondence2D3D> vCorr;
		for (int iPoint = 0; iPoint < nPoint; iPoint++)
		{
			int id1, id2;
			fin >> id1 >> id2;
			double u, v, X, Y, Z;
			fin >> u >> v >> X >> Y >> Z;
			Correspondence2D3D corr;
			corr.u = u;
			corr.v = v;
			corr.x = X;
			corr.y = Y;
			corr.z = Z;
			corr.id_2D = id1;
			corr.id_3D = id2;
			vCorr.push_back(corr);
		}
		vvCorr.push_back(vCorr);

	}
	vvCorr.pop_back();
	fin.close();
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

void SaveAbsoluteCameraData(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, vector<CvMat *> vK)
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
		cvInvert(vK[iP], invK);
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

void SaveAbsoluteCameraData_AD(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, CvMat *K, vector<vector<int> > vvInlier)
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

		fout << vvInlier[iP].size() << " ";
		for (int iIn = 0; iIn < vvInlier[iP].size(); iIn++)
		{
			fout << vvInlier[iP][iIn] << " ";
		}
		fout << endl;

		cvReleaseMat(&R);
		cvReleaseMat(&t);
		cvReleaseMat(&invK);
		cvReleaseMat(&temp33);
		cvReleaseMat(&temp34);
		cvReleaseMat(&invR);
	}
	
	fout.close();
}

void SaveAbsoluteCameraData_AD(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, vector<CvMat *> vK, vector<vector<int> > vvInlier)
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
		cvInvert(vK[iP], invK);
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

		fout << vvInlier[iP].size() << " ";
		for (int iIn = 0; iIn < vvInlier[iP].size(); iIn++)
		{
			fout << vvInlier[iP][iIn] << " ";
		}
		fout << endl;

		cvReleaseMat(&R);
		cvReleaseMat(&t);
		cvReleaseMat(&invK);
		cvReleaseMat(&temp33);
		cvReleaseMat(&temp34);
		cvReleaseMat(&invR);
	}
	
	fout.close();
}

void SaveCameraData(string filename, vector<CvMat *> &vC, vector<CvMat *> &vR, vector<int> &vFrame, int nTotalFrame)
{
	ofstream fout;
	fout.open(filename.c_str(), ios_base::out);

	fout << "NumCams: 1" << endl;
	fout << "NumFrames: " << nTotalFrame << endl;
	fout << "NumP: " << vFrame.size() << endl;

	for (int iP = 0; iP < vC.size(); iP++)
	{
		fout << "0 " << vFrame[iP] << endl;
		fout << cvGetReal2D(vC[iP], 0, 0) << " " << cvGetReal2D(vC[iP], 1, 0) << " " << cvGetReal2D(vC[iP], 2, 0) << endl;
		fout << cvGetReal2D(vR[iP], 0, 0) << " " << cvGetReal2D(vR[iP], 0, 1) << " " << cvGetReal2D(vR[iP], 0, 2) << endl;
		fout << cvGetReal2D(vR[iP], 1, 0) << " " << cvGetReal2D(vR[iP], 1, 1) << " " << cvGetReal2D(vR[iP], 1, 2) << endl;
		fout << cvGetReal2D(vR[iP], 2, 0) << " " << cvGetReal2D(vR[iP], 2, 1) << " " << cvGetReal2D(vR[iP], 2, 2) << endl;
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

void LoadDescriptorData_SURF(string filename, vector<vector<double> > &vvDesc)
{
	PrintAlgorithm("Load Descpritor Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int dummy;
	int k = 0;
	while(!fin.eof())
	{
		fin >> dummy;
		fin >> dummy;
		vector<double> vDesc;
		for (int i = 0; i < 64; i++)
		{
			double dummy_d;
			fin >> dummy_d;
			vDesc.push_back(dummy_d);
		}
		vvDesc.push_back(vDesc);
		k++;
		//if (k > 2000)
		//	break;
	}
	vvDesc.pop_back();
	fin.close();
}

void LoadSURFData(string filename, vector<SIFT_Descriptor> &vDescriptor)
{
	PrintAlgorithm("Load SURF Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	int nPoints, nDim;
	file.open(filename.c_str(), ifstream::in);
	file >> nDim >> nPoints;
	//nDim--;

	for (int j = 0; j < nPoints; j++)
	{
		SIFT_Descriptor descriptor;
		double x, y, dummy;
		file >> dummy >> x >> y >> dummy >> dummy >> dummy;
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
		descriptor.vDesc_d = desc;
		descriptor.id = j;

		vDescriptor.push_back(descriptor);
	}
	file.close();
}

void LoadDescriptorData(string filename, vector<vector<int> > &vvDesc)
{
	PrintAlgorithm("Load Descpritor Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int dummy;
	int k = 0;
	while(!fin.eof())
	{
		fin >> dummy;
		fin >> dummy;
		vector<int> vDesc;
		for (int i = 0; i < 128; i++)
		{
			fin >> dummy;
			vDesc.push_back(dummy);
		}
		vvDesc.push_back(vDesc);
		k++;
		//if (k > 2000)
		//	break;
	}
	vvDesc.pop_back();
	fin.close();
}

void LoadDescriptorData_AD(string filename, vector<SIFT_Descriptor_AD> &vSIFT_AD)
{
	PrintAlgorithm("Load Descpritor AD Data");
	cout << "File name: " << filename << endl;
	ifstream fin;
	fin.open(filename.c_str(), ifstream::in);
	int dummy;
	int k = 1;
	while(!fin.eof())
	{
		fin >> dummy;
		int nPoints;
		fin >> nPoints;
		SIFT_Descriptor_AD desc;
		for (int iPoint = 0; iPoint < nPoints; iPoint++)
		{
			double scale, orientation;
			fin >> scale >> orientation;
			vector<int> vDesc;
			for (int i = 0; i < 128; i++)
			{
				fin >> dummy;
				vDesc.push_back(dummy);
			}
			desc.vvDesc.push_back(vDesc);
			desc.vScale.push_back(scale);
			desc.vOrientation.push_back(orientation);
		}
		vSIFT_AD.push_back(desc);
		k++;
		//if (k > 2)
		//	break;
	}
	vSIFT_AD.pop_back();
	fin.close();
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

void SaveVisibleSetData(string filename, vector<vector<int> > vVisibleSet)
{
	PrintAlgorithm("Save Structure Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	for (int i = 0; i < vVisibleSet.size(); i++)
	{
		fout << vVisibleSet[i].size() << " ";
		for (int j = 0; j < vVisibleSet[i].size(); j++)
		{
			fout << vVisibleSet[i][j] << " ";
		}
		fout << endl;
	}
	fout.close();
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
			for (int id = 0; id < 128; id++)
			{
				file << vFeature[iFeature].vvDesc[i][id] << " ";
			}
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
			file << vFeature[iFeature].vCamera[i] << " " << (int) (vFeature[iFeature].vFrame[i]-vFeature[iFeature].vCamera[i]*nFrames) << " " << vFeature[iFeature].vx[i] << " " << vFeature[iFeature].vy[i] << " ";
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
				file << vFeature[iFeature].vvDesc[id][i] << " ";
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

void SaveCameraRayData(string filename, Camera camera, double bandwidth, int nFrames)
{
	PrintAlgorithm("Save Camera Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumCams: 1 " << endl;
	fout << "NumFrames: " << nFrames << endl;
	fout << "NumP: " << camera.vTakenFrame.size() << endl;
	fout << "Bandwidth: " << bandwidth << endl;

	for (int iFrame = 0; iFrame < camera.vTakenFrame.size(); iFrame++)
	{
		fout << "0 " << camera.vTakenFrame[iFrame] << endl;
		fout << cvGetReal2D(camera.vC[iFrame], 0, 0) << " " << cvGetReal2D(camera.vC[iFrame], 1, 0) << " " << cvGetReal2D(camera.vC[iFrame], 2, 0) << endl;
		fout << cvGetReal2D(camera.vR[iFrame], 0, 0) << " " << cvGetReal2D(camera.vR[iFrame], 0, 1) << " " << cvGetReal2D(camera.vR[iFrame], 0, 2) << endl;
		fout << cvGetReal2D(camera.vR[iFrame], 1, 0) << " " << cvGetReal2D(camera.vR[iFrame], 1, 1) << " " << cvGetReal2D(camera.vR[iFrame], 1, 2) << endl;
		fout << cvGetReal2D(camera.vR[iFrame], 2, 0) << " " << cvGetReal2D(camera.vR[iFrame], 2, 1) << " " << cvGetReal2D(camera.vR[iFrame], 2, 2) << endl;

		fout << cvGetReal2D(camera.vRayCenter[iFrame], 0, 0) << " " << cvGetReal2D(camera.vRayCenter[iFrame], 1, 0) << " " << cvGetReal2D(camera.vRayCenter[iFrame], 2, 0) << endl;
		fout << cvGetReal2D(camera.vRayDirection[iFrame], 0, 0) << " " << cvGetReal2D(camera.vRayDirection[iFrame], 1, 0) << " " << cvGetReal2D(camera.vRayDirection[iFrame], 2, 0) << endl;
	}

	fout.close();
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

void LoadRelativeTransformData(string filename, vector<vector<int> > &vvFrame1, vector<vector<int> > &vvFrame2,
							   vector<vector<CvMat *> > &vvM, vector<vector<CvMat *> > &vvm)
{
	PrintAlgorithm("Load Relative Transform Data");
	ifstream file;
	file.open(filename.c_str(), ifstream::in);
	while (!file.eof())
	{
		string dummy;
		int nPairs;
		file >> dummy >> nPairs;
		if (strcmp(dummy.c_str(), "NumPairs:"))
			break;
		vector<int> vFrame1, vFrame2;
		vector<CvMat *> vM, vm;
		for (int iPair = 0; iPair < nPairs; iPair++)
		{
			int frame1, frame2;
			double m1, m2, m3;
			double M11, M12, M13, M21, M22, M23, M31, M32, M33;
			file >> frame1 >> frame2;
			file >> m1 >> m2 >> m3;
			file >> M11 >> M12 >> M13;
			file >> M21 >> M22 >> M23;
			file >> M31 >> M32 >> M33;

			CvMat *m = cvCreateMat(3,1,CV_32FC1);
			CvMat *M = cvCreateMat(3,3,CV_32FC1);

			cvSetReal2D(m, 0, 0, m1);		cvSetReal2D(m, 1, 0, m2);		cvSetReal2D(m, 2, 0, m3);
			cvSetReal2D(M, 0, 0, M11);		cvSetReal2D(M, 0, 1, M12);		cvSetReal2D(M, 0, 2, M13);
			cvSetReal2D(M, 1, 0, M21);		cvSetReal2D(M, 1, 1, M22);		cvSetReal2D(M, 1, 2, M23);
			cvSetReal2D(M, 2, 0, M31);		cvSetReal2D(M, 2, 1, M32);		cvSetReal2D(M, 2, 2, M33);

			vFrame1.push_back(frame1);
			vFrame2.push_back(frame2);
			vM.push_back(M);
			vm.push_back(m);
		}
		vvFrame1.push_back(vFrame1);
		vvFrame2.push_back(vFrame2);
		vvM.push_back(vM);
		vvm.push_back(vm);
	}
	file.close();
}

void LoadBundlerListData(string filename, Camera &cam)
{
	PrintAlgorithm("Load Bundler File List Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	for (int iFrame = 0; iFrame < cam.vC.size(); iFrame++)
	{
		string dummy;
		file >> dummy;
		if (dummy.size() == 1)
		{
			file >> dummy;
			file >> dummy;
		}
		

		//char temp[1000];
		//file.getline(temp, 1000);
		//int i = 0;
		//while (temp[i] != ' ')
		//{
		//	a.push_back(temp[i]);
		//}

		cam.vFileName.push_back(dummy);		
	}

	file.close();
}

void LoadBundlerResolutionData(string filename, Camera &cam)
{
	PrintAlgorithm("Load Bundler File List Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	for (int iFrame = 0; iFrame < cam.vK.size(); iFrame++)
	{
		string dummy;
		file >> dummy;
		cam.vFileName.push_back(dummy);	
		int w, h;
		file >> w >> h;
		cam.vImageWidth.push_back(w);
		cam.vImageHeight.push_back(h);
	}

	file.close();
}

void LoadBundlerResolutionData(string filename, vector<string> &vFileName)
{
	PrintAlgorithm("Load Bundler File List Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	while (!file.eof())
	{
		string dummy;
		file >> dummy;
		vFileName.push_back(dummy);	
		int w, h;
		file >> w >> h;
	}
	vFileName.pop_back();
	file.close();
}

void LoadBundlerOutputData(string filename, Camera &cam, vector<StaticStructure> &vSS)
{
	PrintAlgorithm("Load Bundler Output File Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	char temp[1000];
	file.getline(temp, 1000);
	
	int nCameras, nPoints;
	file >> nCameras >> nPoints;

	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		double f, k1, k2;
		double t1, t2, t3;
		double r11, r12, r13, r21, r22, r23, r31, r32, r33;

		file >> f >> k1 >> k2;
		file >> r11 >> r12 >> r13;
		file >> r21 >> r22 >> r23;
		file >> r31 >> r32 >> r33;
		file >> t1 >> t2 >> t3;

		if (f == 0)
			continue;

		CvMat *t = cvCreateMat(3,1, CV_32FC1);
		CvMat *R = cvCreateMat(3,3, CV_32FC1);
		CvMat *C = cvCreateMat(3,1, CV_32FC1);
		CvMat *K = cvCreateMat(3,3, CV_32FC1);
		cvSetIdentity(K);
		cvSetReal2D(K, 0, 0, f);
		cvSetReal2D(K, 1, 1, f);

		CvMat *Rt = cvCreateMat(3,3,CV_32FC1);
		cvSetIdentity(Rt);
		cvSetReal2D(Rt, 1, 1, -1);
		cvSetReal2D(Rt, 2, 2, -1);

		cvSetReal2D(t, 0, 0, t1);
		cvSetReal2D(t, 1, 0, t2);
		cvSetReal2D(t, 2, 0, t3);

		cvSetReal2D(R, 0, 0, r11);	cvSetReal2D(R, 0, 1, r12);	cvSetReal2D(R, 0, 2, r13);
		cvSetReal2D(R, 1, 0, r21);	cvSetReal2D(R, 1, 1, r22);	cvSetReal2D(R, 1, 2, r23);
		cvSetReal2D(R, 2, 0, r31);	cvSetReal2D(R, 2, 1, r32);	cvSetReal2D(R, 2, 2, r33);

		CvMat *R_tr = cvCreateMat(3,3,CV_32FC1);
		cvTranspose(R, R_tr);
		cvMatMul(R_tr, t, C);
		ScalarMul(C, -1, C);

		cvMatMul(Rt, R, R);

		cam.vC.push_back(C);
		cam.vR.push_back(R);
		cam.vk1.push_back(k1);
		cam.vk2.push_back(k2);
		cam.vK.push_back(K);
		cvReleaseMat(&Rt);
		cvReleaseMat(&R_tr);
	}

	cout << "Camera Done!!" << endl;
	//return;

	for (int iPoint = 0; iPoint < nPoints; iPoint++)
	{
		if (iPoint % 50000 == 0)
			cout << iPoint << " " << nPoints << endl;
		double x, y, z;
		int r, g, b;
		int nVisible;
		int camera_id;
		int point_id;
		StaticStructure ss;

		file >> x >> y >> z;
		file >> r >> g >> b;
		file >> nVisible;

		for (int iVisible = 0; iVisible < nVisible; iVisible++)
		{
			double dummy;
			file >> camera_id >> point_id >> dummy >> dummy;
			ss.vVisibleCamera.push_back(camera_id);
			ss.vVisibleSIFTPoint.push_back(point_id);
		}
		ss.x = x;
		ss.y = y;
		ss.z = z;
		ss.r = r;
		ss.g = g;
		ss.b = b;
		ss.id = iPoint;	

		vSS.push_back(ss);
	}
	file.close();
}

void LoadNVMOutputData(string filename, Camera &cam, vector<StaticStructure> &vSS, int res_x, int res_y)
{
	PrintAlgorithm("Load NVM Output File Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	char temp[1000];
	file.getline(temp, 1000);
	
	int nCameras, nPoints;
	file >> nCameras;

	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		double f, k1, k2;
		double t1, t2, t3;
		double q1, q2, q3, q4;
	
		string imagename;
		file >> imagename;

		file >> f;
		file >> q1 >> q2 >> q3 >> q4;
		CvMat *q = cvCreateMat(4,1,CV_32FC1);
		cvSetReal2D(q, 0, 0, q1);
		cvSetReal2D(q, 1, 0, q2);
		cvSetReal2D(q, 2, 0, q3);
		cvSetReal2D(q, 3, 0, q4);
		CvMat *R = cvCreateMat(3,3,CV_32FC1);
		Quaternion2Rotation(q, R);

		file >> t1 >> t2 >> t3;
		file >> k1;
		int dummy_d;
		file >> dummy_d;

		if (f == 0)
			continue;

		CvMat *t = cvCreateMat(3,1, CV_32FC1);
		//CvMat *C = cvCreateMat(3,1, CV_32FC1);
		CvMat *K = cvCreateMat(3,3, CV_32FC1);
		cvSetIdentity(K);
		cvSetReal2D(K, 0, 0, f);
		cvSetReal2D(K, 1, 1, f);
		cvSetReal2D(K, 0, 2, res_x/2);
		cvSetReal2D(K, 1, 2, res_y/2);

		//CvMat *Rt = cvCreateMat(3,3,CV_32FC1);
		//cvSetIdentity(Rt);
		//cvSetReal2D(Rt, 1, 1, -1);
		//cvSetReal2D(Rt, 2, 2, -1);

		cvSetReal2D(t, 0, 0, t1);
		cvSetReal2D(t, 1, 0, t2);
		cvSetReal2D(t, 2, 0, t3);

		//CvMat *R_tr = cvCreateMat(3,3,CV_32FC1);
		//cvTranspose(R, R_tr);
		//cvMatMul(R_tr, t, C);
		//ScalarMul(C, -1, C);

		//cvMatMul(Rt, R, R);

		cam.vC.push_back(t);
		cam.vR.push_back(R);
		cam.vk1.push_back(k1);
		cam.vK.push_back(K);
		cam.vImageName.push_back(imagename);
		cam.vTakenFrame.push_back(iCamera);
		//cvReleaseMat(&Rt);
		//cvReleaseMat(&R_tr);
	}

	cout << "Camera Done!!" << endl;
	//return;
	file >> nPoints;
	for (int iPoint = 0; iPoint < nPoints; iPoint++)
	{
		double x, y, z;
		int r, g, b;
		int nVisible;
		int camera_id;
		int point_id;
		StaticStructure ss;

		file >> x >> y >> z;
		file >> r >> g >> b;
		file >> nVisible;

		for (int iVisible = 0; iVisible < nVisible; iVisible++)
		{
			double dummy;
			file >> camera_id >> point_id >> dummy >> dummy;
			//ss.vVisibleCamera.push_back(camera_id);
			//ss.vVisibleSIFTPoint.push_back(point_id);
		}
		ss.x = x;
		ss.y = y;
		ss.z = z;
		ss.r = r;
		ss.g = g;
		ss.b = b;
		//ss.id = iPoint;	

		vSS.push_back(ss);
	}
	file.close();
}


void LoadBundlerOutputData(string filename, Camera &cam)
{
	PrintAlgorithm("Load Bundler Output File Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	char temp[1000];
	file.getline(temp, 1000);
	
	int nCameras, nPoints;
	file >> nCameras >> nPoints;

	for (int iCamera = 0; iCamera < nCameras; iCamera++)
	{
		double f, k1, k2;
		double t1, t2, t3;
		double r11, r12, r13, r21, r22, r23, r31, r32, r33;

		file >> f >> k1 >> k2;
		file >> r11 >> r12 >> r13;
		file >> r21 >> r22 >> r23;
		file >> r31 >> r32 >> r33;
		file >> t1 >> t2 >> t3;

		CvMat *t = cvCreateMat(3,1, CV_32FC1);
		CvMat *R = cvCreateMat(3,3, CV_32FC1);
		CvMat *C = cvCreateMat(3,1, CV_32FC1);
		CvMat *K = cvCreateMat(3,3, CV_32FC1);
		cvSetIdentity(K);
		cvSetReal2D(K, 0, 0, f);
		cvSetReal2D(K, 1, 1, f);

		CvMat *Rt = cvCreateMat(3,3,CV_32FC1);
		cvSetIdentity(Rt);
		cvSetReal2D(Rt, 1, 1, -1);
		cvSetReal2D(Rt, 2, 2, -1);

		cvSetReal2D(t, 0, 0, t1);
		cvSetReal2D(t, 1, 0, t2);
		cvSetReal2D(t, 2, 0, t3);

		cvSetReal2D(R, 0, 0, r11);	cvSetReal2D(R, 0, 1, r12);	cvSetReal2D(R, 0, 2, r13);
		cvSetReal2D(R, 1, 0, r21);	cvSetReal2D(R, 1, 1, r22);	cvSetReal2D(R, 1, 2, r23);
		cvSetReal2D(R, 2, 0, r31);	cvSetReal2D(R, 2, 1, r32);	cvSetReal2D(R, 2, 2, r33);

		CvMat *R_tr = cvCreateMat(3,3,CV_32FC1);
		cvTranspose(R, R_tr);
		cvMatMul(R_tr, t, C);
		ScalarMul(C, -1, C);

		cvMatMul(Rt, R, R);

		cam.vC.push_back(C);
		cam.vR.push_back(R);
		cam.vk1.push_back(k1);
		cam.vk2.push_back(k2);
		cam.vK.push_back(K);
		cvReleaseMat(&Rt);
		cvReleaseMat(&R_tr);
	}

	cout << "Camera Done!!" << endl;
	//return;

	//for (int iPoint = 0; iPoint < nPoints; iPoint++)
	//{
	//	if (iPoint % 50000 == 0)
	//		cout << iPoint << " " << nPoints << endl;
	//	double x, y, z;
	//	int r, g, b;
	//	int nVisible;
	//	int camera_id;
	//	int point_id;
	//	StaticStructure ss;

	//	file >> x >> y >> z;
	//	file >> r >> g >> b;
	//	file >> nVisible;

	//	for (int iVisible = 0; iVisible < nVisible; iVisible++)
	//	{
	//		double dummy;
	//		file >> camera_id >> point_id >> dummy >> dummy;
	//		ss.vVisibleCamera.push_back(camera_id);
	//		ss.vVisibleSIFTPoint.push_back(point_id);
	//	}
	//	ss.x = x;
	//	ss.y = y;
	//	ss.z = z;
	//	ss.r = r;
	//	ss.g = g;
	//	ss.b = b;
	//	ss.id = iPoint;	

	//	vSS.push_back(ss);
	//}
	file.close();
}


void SaveStaticStructure(string filename, vector<StaticStructure> vSS)
{
	PrintAlgorithm("Save StaticStructure Data");
	cout << "File name: " << filename << endl;
	ofstream fout;
	fout.open(filename.c_str());
	fout << "NumStructures: " << vSS.size() << endl;
	
	for (int iSS = 0; iSS < vSS.size(); iSS++)
	{
		fout << vSS[iSS].id << " " << vSS[iSS].r << " " << vSS[iSS].g << " " << vSS[iSS].b << " ";
		fout << vSS[iSS].x << " " << vSS[iSS].y << " " << vSS[iSS].z << endl;
	}
	fout.close();
}

void SaveDescriptorData(string filename, vector<SIFT_Descriptor> vSIFT)
{
	ofstream file;
	file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vSIFT.size(); iFeature++)
	{
		file << iFeature << " 1 ";
		for (int id = 0; id < 128; id++)
		{
			file << vSIFT[iFeature].vDesc[id] << " ";
		}
		file << endl;
	}
	file.close();
}

void SaveDescriptorData_double(string filename, vector<SIFT_Descriptor> vSIFT)
{
	ofstream file;
	file.open(filename.c_str(), ios_base::out);
	for (int iFeature = 0; iFeature < vSIFT.size(); iFeature++)
	{
		file << iFeature << " 1 ";
		for (int id = 0; id < 128; id++)
		{
			file << (int) vSIFT[iFeature].vDesc_d[id] << " ";
		}
		file << endl;
	}
	file.close();
}

void SaveHomographyData(string filename, vector<vector<int> > vvFrame, vector<vector<CvMat *> > vvH_im2nim, vector<vector<CvMat *> > vvH_nim2nim,
						vector<vector<double> > vvx, vector<vector<double> > vvy, vector<vector<double> > vvx_dis, vector<vector<double> > vvy_dis,
						vector<vector<double> > vvScale, vector<vector<double> > vvOrientation)
{
	ofstream file;
	file.open(filename.c_str(), ios_base::out);
	file << "NumPoints: " << vvFrame.size() << endl;
	for (int iPoint = 0; iPoint < vvFrame.size(); iPoint++)
	{
		file << "VisibleImages: " << vvFrame[iPoint].size() << endl;
		for (int iFrame = 0; iFrame < vvFrame[iPoint].size(); iFrame++)
		{
			file << "0 " << vvFrame[iPoint][iFrame] << " ";
			file << vvx[iPoint][iFrame] << " " << vvy[iPoint][iFrame] << " " << vvx_dis[iPoint][iFrame] << " " << vvy_dis[iPoint][iFrame] << " ";
			file << vvScale[iPoint][iFrame] << " " << vvOrientation[iPoint][iFrame] << " ";
			file << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 0, 0) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 0, 1) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 0, 2) << " ";
			file << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 1, 0) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 1, 1) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 1, 2) << " ";
			file << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 2, 0) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 2, 1) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 2, 2) << " ";

			file << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 0, 0) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 0, 1) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 0, 2) << " ";
			file << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 1, 0) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 1, 1) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 1, 2) << " ";
			file << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 2, 0) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 2, 1) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 2, 2) << " ";

			file << endl;
		}
	}
	file.close();
}


void SaveHomographyData(string filename, vector<int> vID, vector<vector<int> > vvFrame, vector<vector<CvMat *> > vvH_im2nim, vector<vector<CvMat *> > vvH_nim2nim,
						vector<vector<double> > vvx, vector<vector<double> > vvy, vector<vector<double> > vvx_dis, vector<vector<double> > vvy_dis,
						vector<vector<double> > vvScale, vector<vector<double> > vvOrientation, int mode)
{

	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
	{
		file.open(filename.c_str(), ios_base::out);
		file.close();
		return;
	}
	for (int iPoint = 0; iPoint < vvFrame.size(); iPoint++)
	{
		file << "Point: " << vID[iPoint] <<"VisibleImages: " << vvFrame[iPoint].size() << endl;
		for (int iFrame = 0; iFrame < vvFrame[iPoint].size(); iFrame++)
		{
			file << "0 " << vvFrame[iPoint][iFrame] << " ";
			file << vvx[iPoint][iFrame] << " " << vvy[iPoint][iFrame] << " " << vvx_dis[iPoint][iFrame] << " " << vvy_dis[iPoint][iFrame] << " ";
			file << vvScale[iPoint][iFrame] << " " << vvOrientation[iPoint][iFrame] << " ";
			file << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 0, 0) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 0, 1) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 0, 2) << " ";
			file << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 1, 0) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 1, 1) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 1, 2) << " ";
			file << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 2, 0) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 2, 1) << " " << cvGetReal2D(vvH_im2nim[iPoint][iFrame], 2, 2) << " ";

			file << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 0, 0) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 0, 1) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 0, 2) << " ";
			file << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 1, 0) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 1, 1) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 1, 2) << " ";
			file << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 2, 0) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 2, 1) << " " << cvGetReal2D(vvH_nim2nim[iPoint][iFrame], 2, 2) << " ";

			file << endl;
		}
	}
	file.close();
}

void SavePatchData(string filename, vector<int> vID, vector<vector<int> > vvFrame,
		vector<vector<double> > vvx11, vector<vector<double> > vvx12, vector<vector<double> > vvx21, vector<vector<double> > vvx22,
		vector<vector<double> > vvy11, vector<vector<double> > vvy12, vector<vector<double> > vvy21, vector<vector<double> > vvy22,
		vector<CvMat *> vX11, vector<CvMat *> vX12, vector<CvMat *> vX21, vector<CvMat *> vX22, 
		vector<CvMat *> vPI, vector<CvMat *> vT, int mode)
{

	ofstream file;
	if (mode == FILESAVE_APPEND_MODE)
		file.open(filename.c_str(), ios_base::app);
	else
	{
		file.open(filename.c_str(), ios_base::out);
		file.close();
		return;
	}
	for (int iPoint = 0; iPoint < vvFrame.size(); iPoint++)
	{
		file << "Point " << vID[iPoint] << endl;
		file << cvGetReal2D(vX11[iPoint], 0, 0) << " " << cvGetReal2D(vX11[iPoint], 1, 0) << " " << cvGetReal2D(vX11[iPoint], 2, 0) << endl; 
		file << cvGetReal2D(vX12[iPoint], 0, 0) << " " << cvGetReal2D(vX12[iPoint], 1, 0) << " " << cvGetReal2D(vX12[iPoint], 2, 0) << endl; 
		file << cvGetReal2D(vX21[iPoint], 0, 0) << " " << cvGetReal2D(vX21[iPoint], 1, 0) << " " << cvGetReal2D(vX21[iPoint], 2, 0) << endl; 
		file << cvGetReal2D(vX22[iPoint], 0, 0) << " " << cvGetReal2D(vX22[iPoint], 1, 0) << " " << cvGetReal2D(vX22[iPoint], 2, 0) << endl; 
		file << cvGetReal2D(vPI[iPoint], 0, 0) << " " << cvGetReal2D(vPI[iPoint], 1, 0) << " " << cvGetReal2D(vPI[iPoint], 2, 0) << " " << cvGetReal2D(vPI[iPoint], 3, 0) << endl; 
		file << cvGetReal2D(vT[iPoint], 0, 0) << " " << cvGetReal2D(vT[iPoint], 0, 1) << " " << cvGetReal2D(vT[iPoint], 0, 2) << " " << cvGetReal2D(vT[iPoint], 0, 3) << endl; 
		file << cvGetReal2D(vT[iPoint], 1, 0) << " " << cvGetReal2D(vT[iPoint], 1, 1) << " " << cvGetReal2D(vT[iPoint], 1, 2) << " " << cvGetReal2D(vT[iPoint], 1, 3) << endl; 
		file << cvGetReal2D(vT[iPoint], 2, 0) << " " << cvGetReal2D(vT[iPoint], 2, 1) << " " << cvGetReal2D(vT[iPoint], 2, 2) << " " << cvGetReal2D(vT[iPoint], 2, 3) << endl; 
		file << cvGetReal2D(vT[iPoint], 3, 0) << " " << cvGetReal2D(vT[iPoint], 3, 1) << " " << cvGetReal2D(vT[iPoint], 3, 2) << " " << cvGetReal2D(vT[iPoint], 3, 3) << endl; 

		if (cvGetReal2D(vT[iPoint], 3, 3) != 0)
		{
			file << vvFrame[iPoint].size() << " "; 
			for (int iFrame = 0; iFrame < vvFrame[iPoint].size(); iFrame++)
			{
				file << "0 " << vvFrame[iPoint][iFrame] << " ";
				file << vvx11[iPoint][iFrame] << " " << vvy11[iPoint][iFrame] << " ";
				file << vvx12[iPoint][iFrame] << " " << vvy12[iPoint][iFrame] << " ";
				file << vvx21[iPoint][iFrame] << " " << vvy21[iPoint][iFrame] << " ";
				file << vvx22[iPoint][iFrame] << " " << vvy22[iPoint][iFrame] << " ";
			}
			file << endl;
		}
		else
		{
			file << "0" << endl;
		}

		
	}
	file.close();
}

void LoadPatchData(string filename, int nStructure, vector<int> &vID, vector<vector<int> > &vvFrame,
		vector<vector<double> > &vvx11, vector<vector<double> > &vvx12, vector<vector<double> > &vvx21, vector<vector<double> > &vvx22,
		vector<vector<double> > &vvy11, vector<vector<double> > &vvy12, vector<vector<double> > &vvy21, vector<vector<double> > &vvy22,
		vector<CvMat *> &vX11, vector<CvMat *> &vX12, vector<CvMat *> &vX21, vector<CvMat *> &vX22, 
		vector<CvMat *> &vPI, vector<CvMat *> &vT)
{
	PrintAlgorithm("Load Bundler Output File Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	
	for (int iFeature = 0; iFeature < nStructure; iFeature++)
	{
		string dummy;
		int id;
		file >> dummy >> id;
		vID.push_back(id);
		double x, y, z;
		file >> x >> y >> z;
		CvMat *X11 = cvCreateMat(3,1,CV_32FC1);
		cvSetReal2D(X11, 0, 0, x);	cvSetReal2D(X11, 1, 0, y);	cvSetReal2D(X11, 2, 0, z);
		vX11.push_back(X11);
		file >> x >> y >> z;
		CvMat *X12 = cvCreateMat(3,1,CV_32FC1);
		cvSetReal2D(X12, 0, 0, x);	cvSetReal2D(X12, 1, 0, y);	cvSetReal2D(X12, 2, 0, z);
		vX12.push_back(X12);
		file >> x >> y >> z;
		CvMat *X21 = cvCreateMat(3,1,CV_32FC1);
		cvSetReal2D(X21, 0, 0, x);	cvSetReal2D(X21, 1, 0, y);	cvSetReal2D(X21, 2, 0, z);
		vX21.push_back(X21);
		file >> x >> y >> z;
		CvMat *X22 = cvCreateMat(3,1,CV_32FC1);
		cvSetReal2D(X22, 0, 0, x);	cvSetReal2D(X22, 1, 0, y);	cvSetReal2D(X22, 2, 0, z);
		vX22.push_back(X22);

		double pi1, pi2, pi3, pi4;
		file >> pi1 >> pi2 >> pi3 >> pi4;
		CvMat *pi = cvCreateMat(4,1,CV_32FC1);
		cvSetReal2D(pi, 0, 0, pi1);	cvSetReal2D(pi, 1, 0, pi2);	cvSetReal2D(pi, 2, 0, pi3);	cvSetReal2D(pi, 3, 0, pi4);
		vPI.push_back(pi);

		double t11, t12, t13, t14;
		double t21, t22, t23, t24;
		double t31, t32, t33, t34;
		double t41, t42, t43, t44;

		file >> t11 >> t12 >> t13 >> t14;
		file >> t21 >> t22 >> t23 >> t24;
		file >> t31 >> t32 >> t33 >> t34;
		file >> t41 >> t42 >> t43 >> t44;

		CvMat *T = cvCreateMat(4,4,CV_32FC1);
		cvSetReal2D(T, 0, 0, t11);	cvSetReal2D(T, 0, 1, t12);	cvSetReal2D(T, 0, 2, t13);	cvSetReal2D(T, 0, 3, t14);
		cvSetReal2D(T, 1, 0, t21);	cvSetReal2D(T, 1, 1, t22);	cvSetReal2D(T, 1, 2, t23);	cvSetReal2D(T, 1, 3, t24);
		cvSetReal2D(T, 2, 0, t31);	cvSetReal2D(T, 2, 1, t32);	cvSetReal2D(T, 2, 2, t33);	cvSetReal2D(T, 2, 3, t34);
		cvSetReal2D(T, 3, 0, t41);	cvSetReal2D(T, 3, 1, t42);	cvSetReal2D(T, 3, 2, t43);	cvSetReal2D(T, 3, 3, t44);
		vT.push_back(T);

		int nVisible;
		file >> nVisible;
		vector<double> vx11, vy11, vx12, vy12, vx21, vy21, vx22, vy22;
		vector<int> vFrame;
		for (int iFrame = 0; iFrame < nVisible; iFrame++)
		{
			int dummy, frame;
			double x11, y11, x12, y12, x21, y21, x22, y22;
			file >> dummy >> frame >> x11 >> y11 >> x12 >> y12 >> x21 >> y21 >> x22 >> y22;
			vFrame.push_back(frame);
			vx11.push_back(x11);
			vy11.push_back(y11);
			vx12.push_back(x12);
			vy12.push_back(y12);
			vx21.push_back(x21);
			vy21.push_back(y21);
			vx22.push_back(x22);
			vy22.push_back(y22);			
		}
		vvx11.push_back(vx11);
		vvy11.push_back(vy11);
		vvx12.push_back(vx12);
		vvy12.push_back(vy12);
		vvx21.push_back(vx21);
		vvy21.push_back(vy21);
		vvx22.push_back(vx22);
		vvy22.push_back(vy22);
 	}
	file.close();
}

void LoadIndexData_CP(string filename, vector<int> &vFrequency)
{
	PrintAlgorithm("Load CP File Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	
	while (!file.eof())
	{
		int a;
		file >> a;
		vFrequency.push_back(a);
	}
	vFrequency.pop_back();
	file.close();
}

void LoadVectorData(string filename, vector<double> &vVector)
{
	PrintAlgorithm("Load Vector Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int n;

	file >> a >> n;
	for (int i = 0; i < n; i++)
	{
		double dummy;
		file >> dummy;
		vVector.push_back(dummy);
	}
	file.close();
}

void LoadMaxDistanceElementData(string filename, vector<double> &vMaxDistance, vector<vector<double> > &vvMaxDistanceElement)
{
	PrintAlgorithm("Load MaxDistance Data");
	cout << "File name: " << filename << endl;
	ifstream file;
	string a;
	file.open(filename.c_str(), ifstream::in);
	int n;

	file >> a >> n;
	for (int i = 0; i < n; i++)
	{
		double dist;
		file >> dist;
		vMaxDistance.push_back(dist);
		vector<double> vMaxDistanceElement;
		for (int j = 0; j < 128; j++)
		{
			double ele;
			file >> ele;
			vMaxDistanceElement.push_back(ele);
		}
		vvMaxDistanceElement.push_back(vMaxDistanceElement);
	}
	file.close();
}