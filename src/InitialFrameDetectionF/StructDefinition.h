#ifndef STRUCTDEFINITION_H
#define STRUCTDEFINITION_H

#include <vector>
#include <string>
#include <cv.h>
#include <cxcore.h>
#include <stdio.h>
#include <stdlib.h>
//#include <flann/flann.hpp>
using namespace std;

struct FaceReconstruction
{
	vector<int> vPanel, vCamera;
	vector<double> vX, vY, vZ;
	double px, py, pz;
	double nx, ny, nz;

	int id;
	vector<vector<double> > vvu, vvv;
};

struct DomeCamera
{
	CvMat *K; 
	CvMat *C;
	CvMat *R;
	double dist1, dist2;
	CvMat *P;
};

struct Face
{
	int panel, camera;
	vector<double> vx, vy;
};

struct SIFT_Descriptor
{
	int id;
	double r, g, b;
	double x, y;
	double dis_x, dis_y;
	vector<int> vDesc;
	vector<double> vDesc_d;
	double orientation, scale;
};

struct SIFT_Descriptor_AD
{
	int id;
	double r, g, b;
	double x, y;
	double dis_x, dis_y;
	vector<vector<int> > vvDesc;
	vector<double> vScale;
	vector<double> vOrientation;
};

struct POI
{
	int id;
	vector<double> vx, vy, vz;
};

struct POI_Matches
{
	vector<int> vIdx;
	vector<double> vx, vy, vz;
	vector<vector<double> > vvWeight;
	vector<int> vFrame;
	vector<vector<double> > vv_a_cov, vv_b_cov, vv_l_cov;
	vector<double> v_f;
};

struct POI_MeanShift
{
	int frame;
	vector<double> vx, vy, vz;
	vector<CvMat *> v_a;
	vector<CvMat *> v_b;
	vector<CvMat *> v_l;
	vector<double> vf;
};

struct Correspondence2D3D
{
	int id_2D, id_3D;
	double u, v, x, y, z;
};

struct SIFT_Descriptor3D
{
	double x, y, z;
	vector<int> vDesc;
};

struct Feature
{
	double r, g, b;
	bool isAdded;
	bool isRegistered;
	int id;
	int nProj;
	vector<int> vCamera;
	vector<int> vFrame;
	vector<int> vVisibleFrame;
	vector<double> vVisible_x, vVisible_y;
	vector<double> vVisible_dis_x, vVisible_dis_y;
	vector<double> vx, vy;
	vector<double> vx_dis, vy_dis;
	vector<CvMat *> vNullSpace;
	vector<vector<int> > vvDesc;
	vector<int> vIsUsed;
};

struct Theta
{
	int id;
	vector<double> thetaX, thetaY, thetaZ;
	bool isStatic;
	double r, g, b;
	int nBasis;
};
struct FileName 
{
	string path;
	string prefix;
	string suffix;
	string extension;
	int nDigit;
};

struct DynamicStructureTexture
{
	vector<vector<unsigned char> > vTexture;
	vector<int> vHeight;
	vector<int> vWidth;
	vector<double> vScale;
	vector<int> p1id, p2id, p3id;
};

struct DynamicStructureTextures
{
	vector<DynamicStructureTexture> vDST;
};

struct CornerPoint
{
	vector<double> vx, vy;
	int adjacentCameraID;
};

struct Camera
{
	FileName filename;
	FileName filename_d;
	int id;
	int nFrames;
	int first;
	int stride;
	vector<CvMat *> vP;
	vector<int> vFrame;
	vector<int> vTakenFrame;
	vector<double> vTakenInstant;
	vector<string> vImageName;
	CvMat *K;
	double k1;
	double k2;
	vector<CvMat *> vK;
	vector<double> vk1;
	vector<double> vk2;
	vector<double> vFocalLength;
	vector<double> vpx;
	vector<double> vpy;
	string exifFileName;
	string calibrationFilename;
	int timeOffset;
	double ccd_width;
	double ccd_height;
	vector<CvMat *> vC;
	vector<CvMat *> vR;

	vector<string> vTextureFilename;
	vector<string> vFileName;
	vector<vector<unsigned char> > vTexture;
	vector<int> vResizedTextureImageWidth;
	vector<int> vResizedTextureImageHeight;
	vector<vector<unsigned char> > vTexture_low;
	vector<double> vLowScale;

	vector<int> vTextureImageWidth;
	vector<int> vTextureImageHeight;
	vector<IplImage *> vImage_cv;
	vector<double> vTextureScale;
	vector<DynamicStructureTextures> vDynamicStructureTextures;
	vector<DynamicStructureTexture> vDynamicStructureTexture;
	vector<int> vDynamicStructureTextureBB_ulx;
	vector<int> vDynamicStructureTextureBB_uly;
	vector<int> vDynamicStructureTextureBB_lrx;
	vector<int> vDynamicStructureTextureBB_lry;


	int camera_color_r;
	int camera_color_g;
	int camera_color_b;

	vector<CvMat *> vRayCenter;
	vector<CvMat *> vRayDirection;
	double rayBandwidth;

	vector<vector<Correspondence2D3D> > vvCorr;

	vector<int> vImageWidth;
	vector<int> vImageHeight;

	vector<double> vOmega, vpx1, vpy1;

	vector<CornerPoint> vCornerPoint;

	double omega;
	double dis_x, dis_y;
};

struct Point
{
	double x, y;
};

struct Point3D
{
	double x, y, z;
};

struct Descriptor
{
	float desc[128];
};

struct ImageSIFT
{
	IplImage *image;
	vector<Point> vPoint;
	vector<Descriptor> vDescriptor;
};

struct FrameCamera
{
	int takenFrame;
	int frameIdx;
	int cameraID;
	string imageFileName;
	string imageFileName_d;
	ImageSIFT imageSIFT;
	struct feature* featureStruct;
	int nFeatureStruct;

	vector<SIFT_Descriptor> vSift_desc;
	//flann_index_t flannIndex;
	//float *desc;
	//cv::Mat descM;
	//cv::flann::Mat<float> descM;
	//struct FLANNParameters p;

	//vector<flann::Index<flann::L2<float>>> flannIndex;
};

struct DynamicObjectWindow
{
	string originalImage;
	double x1, y1, x2, y2;
	string croppedImage;
};

struct ExifData
{
	int iCamera;
	int iFrame;
	int time;
};

struct AdditionalData 
{
	vector<double *> vIntrinsic;
	vector<int> vUsedFrame;
	double *XYZ;
	bool isStatic;
	int nFrames;
	int nBase;
	int nCameraParameters, nImagePointPrameraters;
	int nFeature_static;
	double nNZ;
	CvMat *visibilityMask;
	int max_nFrames;
	int nFeatures;
	vector<CvMat *> vP;
	vector<CvMat *> vK;
	vector<CvMat *> vx;
	vector<int> vIdx;

	vector<vector<CvMat *> > vvP;
	vector<double> vdP;
	vector<Theta> vTheta;
	vector<double> measurements;
	double *ptr;
	double lambda_t;

	vector<CvMat *> vV;
	vector<double> vBandwidth;
	vector<double> vWeight;

	vector<int> vFrame1, vFrame2, vFrame1_r, vFrame2_r;
	vector<CvMat *> *vx1, *vx2, *vx1_r, *vx2_r;
	double qw, qx, qy, qz;
	vector<double> *vx1_a, *vy1_a, *vx2_a, *vy2_a;

	vector<double> vR11, vR12, vR13, vR21, vR22, vR23, vR31, vR32, vR33;
	vector<double> vC1, vC2, vC3;
	double X, Y, Z;

	vector<vector<CvMat *> > vvP_poi;
	vector<vector<CvMat *> > vvV_poi;
	vector<vector<double> > vvBandwidth;

	vector<vector<vector<Correspondence2D3D> > > vvvCorr;
	vector<vector<int> > vvCameraIndex_CfM;
	vector<double> vMeasurement;
	int stride;

	vector<double> vOmega, vpx1, vpy1;
	vector<double> vk1;

	vector<double> vx1d, vy1d;
	vector<double> vx3d, vy3d;

	vector<vector<CornerPoint> > vvcp; 
	double lambda;

	vector<vector<double> > vVx1, vVy1, vVx2, vVy2;
};

struct StaticStructure 
{
	double x, y, z;
	int r, g, b;
	int id;

	vector<int> vVisibleCamera;
	vector<int> vVisibleSIFTPoint;
};

struct DynamicStructure 
{
	vector<double> vx, vy, vz;
	int r, g, b;
	int id;
};

struct ShapeStructure
{
	vector<double> vx, vy, vz;
	vector<int> vr,vg,vb;
	vector<int> vid;
};

struct CameraStructure
{
	vector<bool> vIsCanonical;
	int frame;
	vector<int> vID;
	vector<string> vCameraName;
	vector<string> vCalibrationFileName;
	vector<double> vDistortionOmega;
	vector<string> vHomographyFileName;
	vector<CvMat *> vC;
	vector<CvMat *> vR;
	vector<CvMat *> vK;
	vector<CvMat *> vCorr2D;
	vector<CvMat *> vCorr3D;
	vector<int> vIsGood;
	vector<CvMat *> vHomoMeasurement1;
	vector<CvMat *> vHomoMeasurement2;
};

#endif // STRUCTDEFINITION_H