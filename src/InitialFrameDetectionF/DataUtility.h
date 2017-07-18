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
void SaveFaceReconstructionData(string filename, vector<vector<FaceReconstruction> > vvFaceReconstruction);
void LoadFaceRefinedMeasurementData(string filename, vector<vector<FaceReconstruction> > &vvFaceReconstruction);
void LoadSocialFaceData(string filename, vector<vector<double> > &vvx, vector<vector<double> > &vvy, vector<vector<double> > &vvz);
void SaveFaceData_Direction(string filename, vector<vector<Face> > vvFace, vector<vector<double> > vvFace_x, vector<vector<double> > vvFace_y, vector<vector<double> > vvFace_z);
void LoadFaceData(string filename, vector<FaceReconstruction> &vFaceReconstruction);
void SaveFaceData(string filename, vector<vector<Face> > vvFace, vector<vector<double> > vvFace_x, vector<vector<double> > vvFace_y, vector<vector<double> > vvFace_z);
void SaveStructureData(string filename, vector<double> vX, vector<double> vY, vector<double> vZ);
void LoadDomeCameraData(string intrinsic, string extrinsic, DomeCamera &dome_camera);
void LoadFaceDetectionData(string filename, vector<Face> &vFace);
void LoadCameraData(string filename, Camera &camera);
void LoadNVMOutputData(string filename, Camera &cam, vector<StaticStructure> &vSS, int res_x, int res_y);
void SaveCameraInfoData(string filename, Camera camera);
void LoadFileList(string filename, vector<string> vFileList);
void LoadSIFTData(string filename, vector<SIFT_Descriptor> &vDescriptor);
void LoadInitialFileData(string filename, int &frame1, int &frame2, int &nFrames);
void LoadDescriptorData(string filename, vector<vector<int> > &vvDesc);
void LoadStructureData(string filename, vector<int> &vID, vector<double> &vx, vector<double> &vy, vector<double> &vz);
void LoadFileListData(string filename, vector<string> &vFilename);
void SaveCorrespondence2D3DData(string filename, vector<Correspondence2D3D> vCorr, int frame, int mode);
void ResaveCorrespondence2D3DData(string filename, int nFiles);
//void SaveAbsoluteCameraData(string filename, int frame, CvMat *P, CvMat *K, int mode);
void ResaveAbsoluteCameraData(string filename, int nFrames);
void SaveAbsoluteCameraData(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, CvMat *K);

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
void LoadCameraData_Ray(string filename, Camera &camera);
void LoadPOIData(string filename, vector<POI> &vPOI);
void SaveRelativePoseData(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm, vector<Feature> &vFeature, vector<vector<int> > &vvVisibleID);
void LoadRelativePoseData(string filename, vector<int> &vFrame1, vector<int> &vFrame2, vector<CvMat*> &vM, vector<CvMat *> &vm, vector<CvMat *> &vx1, vector<CvMat *> &vx2);
void SaveRelativePoseData(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm, vector<CvMat *> &vx1, vector<CvMat *> &vx2);
void SaveRelativePoseData(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm);
void SaveRelativePoseDataAll(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm, int mode);
void SaveRelativePoseDataAll(string filename, vector<int> vFrame1, vector<int> vFrame2, vector<CvMat*> vM, vector<CvMat *> vm, vector<CvMat *> &vx1, vector<CvMat *> &vx2, int mode);
void LoadRelativeTransformData(string filename, vector<vector<int> > &vvFrame1, vector<vector<int> > &vvFrame2,
							   vector<vector<CvMat *> > &vvM, vector<vector<CvMat *> > &vvm);
void SaveCameraData(string filename, vector<CvMat *> &vC, vector<CvMat *> &vR, vector<int> &vFrame, int nTotalFrame);
void LoadCameraData(string filename, Camera &camera, int &nTotalFrames);
void LoadCameraData_Ray(string filename, Camera &camera, int &nFrames);
void SavePOI_Meanshift(string filename, vector<vector<CvMat *> > vvPOI, int nSegments_cov, vector<vector<CvMat *> > vv_a, vector<vector<CvMat *> > vv_b, vector<vector<CvMat *> > vv_l, vector<vector<double> > vvf, vector<int> vFrame);
void LoadPOIData_MeanShift(string filename, vector<POI_MeanShift> &vPOI, int &nSegments_cov);
void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast_Interpolation(string filename, vector<Camera> &vCamera, vector<Feature> &vFeature, int &max_nFrames);
void SaveCameraRayData(string filename, Camera camera, double bandwidth, int nFrames);
void LoadPOIData_MeanShift_GT(string filename, vector<POI_MeanShift> &vPOI);
void CreateEmptyFile(string filename);

void LoadCameraTrajectoryData(string filename, vector<CvMat*> &vC, vector<CvMat*> &vR);
void LoadCameraTrajectoryData(string filename, vector<CvMat*> &vC, vector<CvMat*> &vR, int &endframe);
void SavePOI_Meanshift(string filename, vector<vector<CvMat *> > vvPOI, int nSegments_cov, vector<vector<CvMat *> > vv_a, vector<vector<CvMat*> > vv_b, vector<vector<CvMat *> > vv_l, vector<vector<double> > vvf, vector<int> vFrame,
					   vector<vector<vector<CvMat *> > > vvvMeanTraj);
void LoadPOIData_MeanShift(string filename, vector<POI_MeanShift> &vPOI, int &nSegments_cov, vector<vector<vector<CvMat *> > > &vvvMeanTraj);
void SaveCameraData_KRT_AD(string filename, vector<CvMat*> cP, vector<Camera> vCamera, vector<int> vUsedFrame, int max_nFrames,
	vector<vector<int> > vvPointIndex);
//void LoadPOIData_MeanShift(string filename, vector<POI_MeanShift> &vPOI);
void SaveAbsoluteCameraData_AD(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, CvMat *K, vector<vector<int> > vvInlier);
void LoadCameraData_AD(string filename, Camera &camera, int &nFrames, vector<vector<int> > &vvPointIdx);
void LoadDescriptorData_AD(string filename, vector<SIFT_Descriptor_AD> &vSIFT_AD);
void SaveStructureData_SO(string filename, vector<double> vX, vector<double> vY, vector<double> vZ, 
							vector<double> vX1, vector<double> vY1, vector<double> vZ1,
							vector<int> vR, vector<int> vG, vector<int> vB, vector<int> vID);
void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast_SO(string filename, vector<Feature> &vFeature, vector<Feature> &vUncalibratedFeature);
void LoadBundlerListData(string filename, Camera &cam);
void LoadBundlerOutputData(string filename, Camera &cam, vector<StaticStructure> &vSS);
void SaveCameraData_KRT(string filename, Camera camera);
void SaveStaticStructure(string filename, vector<StaticStructure> vSS);
void SaveDescriptorData(string filename, vector<SIFT_Descriptor> vSIFT);
void SaveHomographyData(string filename, vector<vector<int> > vvFrame, vector<vector<CvMat *> > vvH_im2nim, vector<vector<CvMat *> > vvH_nim2nim,
						vector<vector<double> > vvx, vector<vector<double> > vvy, vector<vector<double> > vvx_dis, vector<vector<double> > vvy_dis,
						vector<vector<double> > vvScale, vector<vector<double> > vvOrientation);
void SaveHomographyData(string filename, vector<int> vID, vector<vector<int> > vvFrame, vector<vector<CvMat *> > vvH_im2nim, vector<vector<CvMat *> > vvH_nim2nim,
						vector<vector<double> > vvx, vector<vector<double> > vvy, vector<vector<double> > vvx_dis, vector<vector<double> > vvy_dis,
						vector<vector<double> > vvScale, vector<vector<double> > vvOrientation, int mode);
void SavePatchData(string filename, vector<int> vID, vector<vector<int> > vvFrame,
		vector<vector<double> > vvx11, vector<vector<double> > vvx12, vector<vector<double> > vvx21, vector<vector<double> > vvx22,
		vector<vector<double> > vvy11, vector<vector<double> > vvy12, vector<vector<double> > vvy21, vector<vector<double> > vvy22,
		vector<CvMat *> vX11, vector<CvMat *> vX12, vector<CvMat *> vX21, vector<CvMat *> vX22, 
		vector<CvMat *> vPI, vector<CvMat *> vT, int mode);

void LoadPatchData(string filename, int nStructure, vector<int> &vID, vector<vector<int> > &vvFrame,
		vector<vector<double> > &vvx11, vector<vector<double> > &vvx12, vector<vector<double> > &vvx21, vector<vector<double> > &vvx22,
		vector<vector<double> > &vvy11, vector<vector<double> > &vvy12, vector<vector<double> > &vvy21, vector<vector<double> > &vvy22,
		vector<CvMat *> &vX11, vector<CvMat *> &vX12, vector<CvMat *> &vX21, vector<CvMat *> &vX22, 
		vector<CvMat *> &vPI, vector<CvMat *> &vT);
void SaveDescriptorData_double(string filename, vector<SIFT_Descriptor> vSIFT);
void LoadBundlerResolutionData(string filename, Camera &cam);
void SaveCameraIntrisicData_CP(string filename, Camera camera);
void LoadBundlerOutputData(string filename, Camera &cam);
void SaveCameraData_KRT_AD(string filename, Camera camera, vector<vector<int> > vvPointIdx);
void LoadBundlerResolutionData(string filename, vector<string> &vFileName);
void LoadCameraIntrisicData_CP(string filename, Camera &camera);
void SaveAbsoluteCameraData(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, vector<CvMat *> vK);
void SaveAbsoluteCameraData_AD(string filename, vector<CvMat *> &vP, vector<int> &vFrame, int nTotalFrame, vector<CvMat *> vK, vector<vector<int> > vvInlier);
void SaveQueryFileData(string filename, Camera camera);
void LoadQueryFileData(string filename, Camera &camera);
//void LoadHomographyData(string filename, vector<vector<int>> &vvFrame, vector<vector<CvMat *>> &vvH_im2nim, vector<vector<CvMat *>> &vvH_nim2nim,
//						vector<vector<double>> &vvx, vector<vector<double>> &vvy, vector<vector<double>> &vvx_dis, vector<vector<double>> &vvy_dis,
//						vector<vector<double>> &vvScale, vector<vector<double>> &vvOrientation);
void LoadIndexData_CP(string filename, vector<int> &vFrequency);
void LoadVectorData(string filename, vector<double> &vVector);
void LoadMaxDistanceElementData(string filename, vector<double> &vMaxDistance, vector<vector<double> > &vvMaxDistanceElement);
void SaveMaxDistanceElementData(string filename, vector<double> vMaxDistance, vector<vector<double> > vvMaxDistanceElement);
void LoadPOIMatchData(string filename, vector<POI_Matches> &vPOI);
void LoadCameraData_Ray(string filename, Camera &camera);
void SavePOI_Matches(string filename, vector<POI_Matches> vPOI_matches);
void LoadCameraTrajectoryData(string filename, vector<CvMat*> &vC, vector<CvMat*> &vR);
void SavePOI_Matches_light(string filename, vector<POI_Matches> vPOI_matches);
void SavePOI_Match_Cov(string filename, vector<POI_Matches> vPOI, int nFrame, int nSegments_cov);
void LoadPOI_Matches_light(string filename, vector<POI_Matches> &vPOI_matches);
void LoadCorrespondence2D3DData(string filename, vector<vector<Correspondence2D3D> > &vvCorr, vector<int> &vFrame);
void LoadStaticMeasurementData_NoCalibration_RGB(string filename, vector<Feature> &vFeature);
void SaveStructureData_RGB(string filename, vector<double> vX, vector<double> vY, vector<double> vZ, vector<int> ID, vector<Feature> vFeature);
void SaveStructureData_RGB_fast_tripple(string filename, CvMat *X, vector<Feature> &vFeature);
void LoadPOI_Match_refined(string filename, vector<POI_Matches> &vPOI, int &nseg);
void SaveCameraData_DistortionK(string filename, vector<Camera> vCamera, vector<CvMat*> cK, vector<double> vk1, vector<int> vUsedFrame, int max_nFrames);
void SaveStructureData_RGB_fast_3More(string filename, CvMat *X, vector<Feature> &vFeature);
void LoadDescriptorData_SURF(string filename, vector<vector<double> > &vvDesc);
void LoadSURFData(string filename, vector<SIFT_Descriptor> &vDescriptor);
void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast_InitialFrame(string filename, vector<Feature> &vFeature, int &max_frame);
void SaveStaticMeasurementData_NoCalibration_RGB_LOWES_fast_InitialFrame(string filename, vector<Feature> vFeature, int max_frame);
void SaveInitialFrame(string filename, int init1, int init2, int max_frame);
void SaveInitialFrameList(string filename, vector<int> init1, vector<int> init2, vector<double> vdist, vector<double> vp, vector<int> vn);
void LoadStaticMeasurementData_NoCalibration_RGB_LOWES_fast(string filename, vector<Feature> &vFeature);
void SaveVisibleSetData(string filename, vector<vector<int> > vVisibleSet);
#endif //DATAUTILITY_H
