#ifndef MATHUTILITY_H
#define MATHUTILITY_H
#include <cv.h>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cmath>
#include "StructDefinition.h"
#define PI 3.14159265
#define QW_ZERO 1e-6
using namespace std;

void GetNormalVector(double ux, double uy, double uz, double vx, double vy, double vz, double &n1, double &n2, double &n3);
void Normalization(CvMat *x0, CvMat *x1, CvMat *T);
void Normalization3D(CvMat *x0, CvMat *x1, CvMat *T);
void Homo2Inhomo(CvMat *xh, CvMat *xi);
void Inhomo2Homo(CvMat *xi, CvMat *xh);
void NormalizingByRow(CvMat *M, int rowIdx);
void NormalizingByCol(CvMat *M, int colIdx);
void xPx_inhomo(CvMat *x, CvMat *P, CvMat *xPx);
void xPx_homo(CvMat *x, CvMat *P, CvMat *xPx);
void xPy_inhomo(CvMat *x, CvMat *y, CvMat *P, CvMat *xPx);
void xPy_homo(CvMat *x, CvMat *y, CvMat *P, CvMat *xPx);
void PrintMat(CvMat *M, string matrixName = "No name");
void PrintMatCol(CvMat *M, int colFrom, int colTo, string matrixName = "No name");
void PrintMatRow(CvMat *M, int rowFrom, int rowTo, string matrixName = "No name");
void GetSubMat(CvMat *M, int rowFrom, int rowTo, int colFrom, int colTo, CvMat *subM);
void SetSubMat(CvMat *M, int rowFrom, int rowTo, int colFrom, int colTo, CvMat *subM);
void SetSubMat(CvMat *M, int rowFrom, int colFrom, CvMat *subM);
void GetSubMatRowwise(CvMat *M, int rowFrom, int rowTo, CvMat *subM);
void GetSubMatColwise(CvMat *M, int colFrom, int colTo, CvMat *subM);
void GetDCTMappingMatrix(CvMat *M, int n);
void GetIDCTMappingMatrix(CvMat *M, int n);
void DCTProjection(CvMat *P, CvMat *Theta, int nFrames, int iFrame, int nBase, CvMat *x);
void GetBiTilde(int nFrames, int iFrame, int nBase, CvMat *BiTilde);
void GetHugeB(int nFrames, int nBase, int nFeatures, CvSparseMat *hugeB);
void GetHugeB(int nFrames, int nBase, int nFeatures, CvMat *hugeB);
void GetHugeQ(int nFrames, int nFeatures, vector<Feature> vFeatures, vector<CvMat *> vCameraMatrix, CvMat &hugeQ, CvMat &hugeq);
void GetTheta(int nFrames, int nFeatures, int nBase, vector<Feature> vFeatures, vector<CvMat *> vCameraMatrix, CvMat &Theta);
void GetThetaWOWeight(int nFrames, int nFeatures, int nBase, vector<Feature> vFeatures, vector<CvMat *> vCameraMatrix, CvMat &Theta);
void GetThetaWOWeight(int nFrames, int nFeatures, int nBase, vector<Feature> vFeatures, vector<Camera> vCamera, CvMat &Theta);
void GetQ(int nFrames, int nBase, int nFeatures, vector<CvMat *> vCameraMatrix, Feature feature, CvMat &Q, CvMat &q);
void GetQ(int nFrames, int nBase, int nFeatures, vector<Camera> vCamera, Feature feature, CvMat &Q, CvMat &q);
void GetBB(int nFrames, int nBase, CvMat &BB);
void Intersection(vector<int> x1, vector<int> x2, vector<int> &intersectionx);
void Union(vector<int> x1, vector<int> x2, vector<int> &unionx);
void ScalarMul(CvMat *M, double v, CvMat *M1);
void LS_homogeneous(CvMat *A, CvMat &x);
void LS_homogeneous(CvMat *A, CvMat *x);
void Pxx_inhomo(CvMat *P, CvMat *x, CvMat &Pxx);
void Pxx_inhomo(CvMat *P, CvMat *x, CvMat *Pxx);
void SetIndexedMatRowwise(CvMat *toM, vector<int> toIndex, CvMat *fromM);
void SetMatRowwiseFromIndexedMat(CvMat *toM, vector<int> fromIndex, CvMat *fromM);
void PrintAlgorithm(string str);
double NormL2(CvMat *M);
void Rotation2Quaternion(CvMat *R, CvMat &q);
void QuaternionNormalization(CvMat *q);
void Quaternion2Rotation(CvMat *q, CvMat &R);
void Quaternion2Rotation(CvMat *q, CvMat *R);
void Rotation2Quaternion(CvMat *R, CvMat *q);
void Vec2Skew(CvMat *m, CvMat *M);
void Homo2InhomoVec(CvMat *xh, CvMat *xi);
void Inhomo2HomoVec(CvMat *xi, CvMat *xh);
double DistancePixel(double x1, double y1, double x2, double y2);
void GetThetaWOWeight(int max_nFrames, int nBase, vector<Feature> vFeatures, vector<Camera> vCamera, CvMat &Theta);
void GetThetaWOWeight_OrthogonalToCamera(int max_nFrames, int nBase, vector<Feature> vFeatures, vector<Camera> vCamera, CvMat &Theta, CvMat &BaseB, CvMat &CameraTraj, bool mode);
void ProjOntoVector(CvMat *v, CvMat *u, CvMat *Projv);
void MatrixOrthogonalization(CvMat *A, CvMat *A1);
void GramSchmidt(CvMat *A, CvMat *v0, CvMat *v1);

void GetCameraParameter(CvMat *P, CvMat *K, CvMat &R, CvMat &C);
void GetCameraParameter(CvMat *P, CvMat *K, CvMat *R, CvMat *C);

//void GetThetaWOWeight(int max_nFrames, int nBase, vector<Feature> vFeatures, vector<Camera> vCamera, CvMat &Theta, CvMat &B);
void GetBi_ortho(vector<CvMat*> vC, CvMat *B, CvMat *Bi);
void CreatCameraMatrix(vector<Camera> vCamera, int max_nFrame, CvMat &CameraMatrix);
void GetB12(int max_nFrame, int nBase, CvMat &B1, CvMat &B2);

double IDCTContinuous(int max_nFrames, double t, vector<double> theta);
void GetIDCTContinuousMatrix(int max_nFrames, int frequency, int nBase, CvMat &BBt);

void GetCameraMatrix(CvMat *K, CvMat *R, CvMat *C, CvMat *P);

void GetThetaWOWeight_basis(int max_nFrames, vector<int> vnBase, vector<Feature> vFeatures, vector<Camera> vCamera, vector<CvMat *> &vTheta);
void SaveVectorData(string filename, vector<double> vVector);

#endif //MATHUTILITY_H