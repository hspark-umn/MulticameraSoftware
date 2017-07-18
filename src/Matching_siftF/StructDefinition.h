#ifndef STRUCTDEFINITION_H
#define STRUCTDEFINITION_H

#include <vector>
#include <string>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <stdio.h>
#include <stdlib.h>
//#include <flann/flann.hpp>
using namespace std;

struct SIFT_Descriptor
{
	int id;
	double r, g, b;
	double x, y;
	double dis_x, dis_y;
	vector<int> vDesc;
	vector<int> vDesc_int;
	double direction;
	double scale;
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
	vector<int> vCamera;
	vector<int> vFrame;
	vector<double> vx, vy;
	vector<double> vx_dis, vy_dis;
	vector<CvMat *> vNullSpace;
	vector<vector<int> > vvDesc_int;
	vector<vector<int> > vvDesc;
	vector<vector<int> > vvDesc_AllDescriptors;
	vector<int> vIsUsed;
	vector<double> vScale;
	vector<double> vDirection;
	vector<int> vStitchedIdx;
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
	vector<string> vImageFileName;
	CvMat *K;
	double k1;
	double k2;
	vector<CvMat *> vK;
	vector<CvMat *> vInvK;
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
};

struct Point
{
	double x, y;
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

	vector<CvMat *> vK;
	vector<CvMat *> vInvK;
	vector<double> vk1;
	vector<double> vk2;
	//flann_index_t flannIndex;
	//float *desc;
	//cv::Mat descM;
	//cv::flann::Mat<float> descM;
	//struct FLANNParameters p;

	//vector<flann::Index<flann::L2<float> >> flannIndex;
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
	vector<double> vdP;
	vector<Theta> vTheta;
	vector<double> measurements;
	double *ptr;
	vector<vector<CvMat *> > vvP_poi;
	vector<vector<CvMat *> > vvV_poi;
	vector<vector<double> > vBandwidth;
	double lambda;
};

struct StaticStructure 
{
	double x, y, z;
	int r, g, b;
	int id;
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

#endif // STRUCTDEFINITION_H