#ifndef DATAUTILITY_H
#define DATAUTILITY_H

#include "MathUtility.h"
#include "StructDefinition.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#define FILESAVE_WRITE_MODE 0
#define FILESAVE_APPEND_MODE 1
#define FEATURE_PROPERTY_DYNAMIC 0
#define FEATURE_PROPERTY_STATIC 1

using namespace std;

void LoadFileList(string filename, vector<string> vFileList);
void LoadSIFTData(string filename, vector<SIFT_Descriptor> &vDescriptor);
void LoadInitialFileData(string filename, int &frame1, int &frame2, int &nFrames);
void LoadDescriptorData(string filename, vector<vector<double> > &vvDesc);
void LoadStructureData(string filename, vector<int> &vID, vector<double> &vx, vector<double> &vy, vector<double> &vz);
void LoadFileListData(string filename, vector<string> &vFilename);
void SaveCorrespondence2D3DData(string filename, vector<Correspondence2D3D> vCorr, int frame, int mode);
void ResaveCorrespondence2D3DData(string filename, int nFiles);
//void SaveAbsoluteCameraData(string filename, int frame, CvMat *P, CvMat *K, int mode);
void ResaveAbsoluteCameraData(string filename, int nFrames);
void SaveAbsoluteCameraData(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, CvMat *K);
void LoadMeasurementData_RGB_DESC_Seq(string filename, vector<Feature> &vFeature);

void LoadMeasurementData(string filename, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras);
void LoadMeasurementData(string filename, CvMat *K, double k1, double k2, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras);
void LoadMeasurementData(string filename, vector<Feature> &vFeature, int &nFeatures, vector<int> &vnFrames, int &nCameras, int &max_nFrames);
void LoadStaticMeasurementData(string filename, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras);
void LoadStaticMeasurementData(string filename, CvMat *K, double k1, double k2, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras, vector<Feature> &vUncalibratedFeature);
void LoadStaticMeasurementData(string filename, CvMat *K, double k1, double k2, vector<Feature> &vFeature, int &nFeatures, int &nFrames, int &nCameras);
void LoadCameraData(string filename, vector<Camera> &vCamera, CvMat *K);
void LoadCameraData(string filename, vector<Camera> &vCamera);

void SaveStructureData(string filename, CvMat *X);
void SaveStructureData(string filename, CvMat *X, vector<int> visibleID);
void SaveCameraData(string filename, vector<CvMat*> cP, CvMat *K);
void SaveCameraData(string filename, vector<CvMat*> cP, CvMat *K, vector<int> vUsedFrame, int nFrames, int nCam);
void SaveCameraData(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames);
void SaveCameraData_KRT(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames);
void SaveMatrix(string filename, CvMat *M);

void LoadCalibrationData(string filename, CvMat &K);
void LoadCalibrationData(string filename, CvMat *K, double &k1, double &k2);
string FilePathGeneration(FileName fn, int number);
void SaveMeasurementData(string filename, vector<Feature> vFeature, int mode);
void SaveMeasurementData(string filename, vector<Feature> vFeature, int mode, double dynamicThreshold);
void SaveMeasurementData(string filename, vector<Feature> vFeature, int nFrames, int mode);
void ResaveMeasurementData(string filename, int nFrames, int nCameras);

void LoadThetaData(string filename, vector<Theta> &vTheta, int &nFeatures, int &nFrames, int &nCameras, int &nBase);
void SaveThetaData(string filename, vector<Theta> vTheta, int nFeatures, int nFrames, int nCameras, int nBase);
void SaveCameraData(string filename, vector<CvMat*> cP, CvMat *K, int nFrames, int nCameras);
void SaveMeasurementData(string filename, vector<Feature> vFeature, int mode, vector<int> visibleID);
void LoadStructureData(string filename, vector<double> &vx, vector<double> &vy, vector<double> &vz);
void SaveCameraData(string filename, vector<Camera> vCamera, CvMat *K, int nFrames, int nCameras);
void LoadDynamicObjectWindow(string filename, string path, vector<DynamicObjectWindow> &vDW);
void LoadDynamicObjectWindow(vector<string> vFilename, vector<string> vPath, vector<DynamicObjectWindow> &vDW);

void LoadCalibrationData(vector<string> vFilename, vector<Camera> &vCamera);
void ResaveMeasurementData(string filename, vector<int> vnFrames, int nCameras);
void LoadExifData(vector<string> vFilename, vector<int> vTimeoffset, vector<Camera> &vCamera);
void LoadStaticMeasurementData(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames);
void SaveCameraData_KRT_intrinsic(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames);
void LoadStaticMeasurementData_NoCalibration(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames);
void LoadFileInfo(string filename, string path, vector<Camera> &vCamera);
void CameraDefinition(string fileinfo, string filepath, string exifFile, string calibrationFile, vector<Camera> &vCamera);
void LoadExifData(vector<Camera> &vCamera);
void LoadCalibrationData(vector<Camera> &vCamera);
void LoadExifDataWithFocalLength(vector<Camera> &vCamera);
void SaveMeasurementData_RGB(string filename, vector<Feature> vFeature, int mode);
void LoadStructureData_Add(string filename, int nFeatures, CvMat &X, vector<int> &vVisibleID);
void LoadCameraIntrinsicData(string filename, vector<Camera> &vCamera);
void LoadCameraData_Add(string filename, vector<Camera> &vCamera, vector<CvMat *> &cP);
void LoadMeasurementData(string filename, vector<Feature> &vFeature, int &nFeatures, vector<int> &vnFrames, int &nCameras, int &max_nFrames, int &max_nCameras);
void LoadCalibrationData(vector<Camera> &vCamera, vector<int> vWOCalibrationCameraID);
void SaveStructureData_RGB(string filename, CvMat *X, vector<int> visibleID, vector<Feature> vFeature);
void LoadMeasurementData_RGB(string filename, vector<Feature> &vFeature, int &nFeatures, vector<int> &vnFrames, int &nCameras, int &max_nFrames, int &max_nCameras);
void SaveMeasurementData_RGB(string filename, vector<Feature> vFeature, int nFrames, int mode);
void LoadStaticMeasurementData_NoCalibration_RGB(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames);
void SaveCameraData_KRT_Tempo(string filename, vector<Camera> vCamera, int max_nFrames);
void SaveCameraData_KRT_TEMPORAL(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames);
void SaveCameraData_KRT_TEMPORAL(string filename, vector<Camera> vCamera, int max_nFrames);
void SaveCameraData_KRT_TEMPORAL(string filename_camera, string filename_intrinsic, vector<Camera> vCamera, int max_nFrames);

// Camera Definition
void CameraDefinition_HandWave(string fileinfo, string filepath, string exifFile, string calibrationFile, vector<Camera> &vCamera);
void CameraDefinition_FunnyMove(string fileinfo, string filepath, string exifFile, string calibrationFile, vector<Camera> &vCamera);

void LoadTimeOffset(string filename, vector<int> &vTimeoffset, vector<int> &vID);
void CameraDefinition(string fileinfo, string filepath, string exifFile, string calibrationFile, string timeoffsetFile, vector<Camera> &vCamera);
void LoadExifDataWithFocalLengthWOK(vector<Camera> &vCamera);
void SaveStructureData(string filename, vector<int> visibleID, vector<double> vx, vector<double> vy, vector<double> vz, vector<int> vr,vector<int> vg,vector<int> vb);
void LoadStructureData(string filename, vector<int> &visibleID, vector<double> &vx, vector<double> &vy, vector<double> &vz, vector<int> &vr,vector<int> &vg,vector<int> &vb);

void LoadCalibrationData_INAUGURATION(string filename, Camera &cam);
void LoadCalibrationData_INAUGURATION(string filename, vector<Camera> &vCamera);
void LoadCameraData_POS(string filename, vector<double> &vx, vector<double> &vy, vector<double> &vz);

void LoadStructureData(string filename, vector<StaticStructure> &vStaticStructure);
void LoadCameraData_TA(string filename, vector<Camera> &vCamera);

void LoadStaticMeasurementData_NoCalibration_RGB(string filename, vector<Camera> &vCamera, vector<Camera> &vCameraTemp, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames);

void SaveMeasurementData_RGB_DESC(string filename, vector<Feature> vFeature, int mode);
void LoadMeasurementData_RGB_DESC(string filename, vector<Feature> &vFeature, int &nFeatures, vector<int> &vnFrames, int &nCameras, int &max_nFrames, int &max_nCameras);
void SaveMeasurementData_DESC(string filename, vector<Feature> vFeature, int nFrames, int mode);
void LoadStaticMeasurementData_NoCalibration_RGB_LOWES(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames);
void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature, int &max_nFrames);
void SaveStructureData_RGB_fast(string filename, CvMat *X, vector<Feature> &vFeature);
void ResaveMeasurementData(string filename, vector<int> vnFrames, int nCameras, int nPoints);
void SaveMeasurementData_RGB_Interpolation(string filename, vector<Feature> vFeature, int mode);
void LoadMeasurementData_RGB_DESC_Seq_Interpolation(string filename, vector<Feature> &vFeature);
void SaveMeasurementData_RGB_DESC1(string filename, vector<Feature> vFeature, int mode);
void LoadMeasurementData_RGB_DESC_Seq1(string filename, vector<Feature> &vFeature);

void LoadMeasurementData_RGB_Seq(string filename, vector<Feature> &vFeature);
void SaveMeasurementData_RGB_NODESC(string filename, vector<Feature> &vFeature, int mode);
void SaveMeasurementData_RGB_NODESC1(string filename, vector<Feature> vFeature, int mode);

void SaveMeasurementData_DESC_AllDescriptor(string filename, vector<Feature> vFeature, int nFrames, int mode);
void LoadSIFTData_ScaleDirection(string filename, vector<SIFT_Descriptor> &vDescriptor);
void SaveMeasurementData_RGB_DESC_ScaleDirection(string filename, vector<Feature> vFeature, int mode);	
void LoadMeasurementData_RGB_DESC_Seq_ScaleDirection(string filename, vector<Feature> &vFeature);
void SaveMeasurementData_DESC_AllDescriptor_ScaleDirection(string filename, vector<Feature> vFeature, int nFrames, int mode);
void LoadDescriptorAllData_MaxDistance(string filename, vector<double> &vMaxDistance);
void SaveVectorData(string filename, vector<double> vVector);
void LoadDescriptorAllData_MaxDistance_Element(string filename, vector<double> &vMaxDistance, vector<vector<double> > &vvMaxDistanceElement);
void SaveMaxDistanceElementData(string filename, vector<double> vMaxDistance, vector<vector<double> > vvMaxDistanceElement);
void LoadSIFTData_subsampling(string filename, vector<SIFT_Descriptor> &vDescriptor);
void LoadSIFTData_int(string filename, vector<SIFT_Descriptor> &vDescriptor);
#endif //DATAUTILITY_H
