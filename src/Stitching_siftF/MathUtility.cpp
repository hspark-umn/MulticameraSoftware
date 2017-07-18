#include "MathUtility.h"
#include <cmath>
#include <math.h>
#include <algorithm>
using namespace std;

void Normalization(CvMat *x0, CvMat *x1, CvMat *T)
{
	double xMean=0, yMean=0, xVar=0, yVar=0;
	for (int i = 0; i < x0->rows; i++)
	{
		xMean += cvGetReal2D(x0, i, 0);
		yMean += cvGetReal2D(x0, i, 1);
	}
	xMean /= x0->rows;
	yMean /= x0->rows;
	for (int i = 0; i < x0->rows; i++)
	{
		xVar += (cvGetReal2D(x0, i, 0)-xMean)*(cvGetReal2D(x0, i, 0)-xMean);
		yVar += (cvGetReal2D(x0, i, 1)-yMean)*(cvGetReal2D(x0, i, 1)-yMean);
	}
	xVar /= x0->rows;
	yVar /= x0->rows;

	double s;
	if ((xVar+yVar) == 0)
		s = 1;
	else
		s = sqrt(2/(xVar+yVar));
	cvSetReal2D(T, 0, 0, s);	cvSetReal2D(T, 0, 1, 0);	cvSetReal2D(T, 0, 2, -s*xMean);
	cvSetReal2D(T, 1, 0, 0);	cvSetReal2D(T, 1, 1, s);	cvSetReal2D(T, 1, 2, -s*yMean);
	cvSetReal2D(T, 2, 0, 0);	cvSetReal2D(T, 2, 1, 0);	cvSetReal2D(T, 2, 2, 1);

	for (int i = 0; i < x0->rows; i++)
	{
		double tx = cvGetReal2D(x0, i, 0);
		double ty = cvGetReal2D(x0, i, 1);

		cvSetReal2D(x1, i, 0, s*tx-s*xMean);
		cvSetReal2D(x1, i, 1, s*ty-s*yMean);
	}
}

void Normalization3D(CvMat *x0, CvMat *x1, CvMat *T)
{
	double xMean=0, yMean=0, zMean=0, xVar=0, yVar=0, zVar=0;
	for (int i = 0; i < x0->rows; i++)
	{
		xMean += cvGetReal2D(x0, i, 0);
		yMean += cvGetReal2D(x0, i, 1);
		zMean += cvGetReal2D(x0, i, 2);
	}
	xMean /= x0->rows;
	yMean /= x0->rows;
	zMean /= x0->rows;
	for (int i = 0; i < x0->rows; i++)
	{
		xVar += (cvGetReal2D(x0, i, 0)-xMean)*(cvGetReal2D(x0, i, 0)-xMean);
		yVar += (cvGetReal2D(x0, i, 1)-yMean)*(cvGetReal2D(x0, i, 1)-yMean);
		zVar += (cvGetReal2D(x0, i, 2)-zMean)*(cvGetReal2D(x0, i, 2)-zMean);
	}
	xVar /= x0->rows;
	yVar /= x0->rows;
	zVar /= x0->rows;

	double s = sqrt(3/(xVar+yVar+zVar));
	cvSetReal2D(T, 0, 0, s);	cvSetReal2D(T, 0, 1, 0);	cvSetReal2D(T, 0, 2, 0);	cvSetReal2D(T, 0, 3, -s*xMean);
	cvSetReal2D(T, 1, 0, 0);	cvSetReal2D(T, 1, 1, s);	cvSetReal2D(T, 1, 2, 0);	cvSetReal2D(T, 1, 3, -s*yMean);
	cvSetReal2D(T, 2, 0, 0);	cvSetReal2D(T, 2, 1, 0);	cvSetReal2D(T, 2, 2, s);	cvSetReal2D(T, 2, 3, -s*zMean);
	cvSetReal2D(T, 3, 0, 0);	cvSetReal2D(T, 3, 1, 0);	cvSetReal2D(T, 3, 2, 0);	cvSetReal2D(T, 3, 3, 1);

	for (int i = 0; i < x0->rows; i++)
	{
		double tx = cvGetReal2D(x0, i, 0);
		double ty = cvGetReal2D(x0, i, 1);
		double tz = cvGetReal2D(x0, i, 2);

		cvSetReal2D(x1, i, 0, s*tx-s*xMean);
		cvSetReal2D(x1, i, 1, s*ty-s*yMean);
		cvSetReal2D(x1, i, 2, s*tz-s*zMean);
	}
}


void Homo2Inhomo(CvMat *xh, CvMat *xi)
{
	for (int i = 0; i < xh->rows; i++)
	{
		for (int j = 0; j < xh->cols-1; j++)
		{
			cvSetReal2D(xi, i, j, cvGetReal2D(xh, i, j)/cvGetReal2D(xh, i, xh->rows-1));
		}
	}
}

void Homo2InhomoVec(CvMat *xh, CvMat *xi)
{
	for (int i = 0; i < xh->rows-1; i++)
	{
		cvSetReal2D(xi, i, 0, cvGetReal2D(xh, i, 0)/cvGetReal2D(xh, xh->rows-1, 0));
	}
}

void Inhomo2Homo(CvMat *xi, CvMat *xh)
{
	for (int i = 0; i < xi->rows; i++)
	{
		for (int j = 0; j < xi->cols; j++)
			cvSetReal2D(xh, i, j, cvGetReal2D(xi, i, j));
		cvSetReal2D(xh, i, xi->cols, 1);
	}
}

void Inhomo2HomoVec(CvMat *xi, CvMat *xh)
{
	for (int i = 0; i < xi->rows; i++)
	{
		cvSetReal2D(xh, i, 0, cvGetReal2D(xi, i, 0));	
	}
	cvSetReal2D(xh, xh->rows-1, 0, 1);
}

void xPx_inhomo(CvMat *x, CvMat *P, CvMat *xPx)
{
	CvMat *y = cvCloneMat(x);
	xPy_inhomo(x, y, P, xPx);
	cvReleaseMat(&y);
}

void xPx_homo(CvMat *xh, CvMat *P, CvMat *xPx)
{
	CvMat *y = cvCloneMat(xh);
	xPy_homo(xh, xh, P, xPx);
	cvReleaseMat(&y);
}

void xPy_inhomo(CvMat *x, CvMat *y, CvMat *P, CvMat *xPy)
{
	CvMat *xh = cvCreateMat(x->rows, x->cols+1, CV_32FC1);
	CvMat *yh = cvCreateMat(x->rows, x->cols+1, CV_32FC1);
	Inhomo2Homo(x, xh);
	Inhomo2Homo(y, yh);
	CvMat *yhT = cvCreateMat(yh->cols, yh->rows, CV_32FC1);
	CvMat *xhP = cvCreateMat(xh->rows, P->cols, CV_32FC1);
	CvMat *xhPyhT = cvCreateMat(xh->rows, yh->rows, CV_32FC1);
	cvTranspose(yh, yhT);

	cvMatMul(xh, P, xhP);
	cvMatMul(xhP, yhT, xhPyhT);	
	for (int i = 0; i < x->rows; i++)
	{
		cvSetReal2D(xPy, i, 0, cvGetReal2D(xhPyhT, i, i));
	}
	cvReleaseMat(&xh);
	cvReleaseMat(&yh);
	cvReleaseMat(&yhT);
	cvReleaseMat(&xhP);
	cvReleaseMat(&xhPyhT);
}

void NormalizingByRow(CvMat *M, int rowIdx)
{
	for (int i = 0 ; i < M->cols; i++)
	{
		double n = cvGetReal2D(M, rowIdx, i);
		for (int j = 0; j < M->rows; j++)
		{
			cvSetReal2D(M, j, i, cvGetReal2D(M, j, i)/n);
		}
	}
}

void NormalizingByCol(CvMat *M, int colIdx)
{
	for (int i = 0 ; i < M->rows; i++)
	{
		double n = cvGetReal2D(M, i, colIdx);
		for (int j = 0; j < M->cols; j++)
		{
			cvSetReal2D(M, i, j, cvGetReal2D(M, i, j)/n);
		}
	}
}

void xPy_homo(CvMat *xh, CvMat *yh, CvMat *P, CvMat *xPy)
{
	CvMat *xhT = cvCreateMat(xh->cols, xh->rows, CV_32FC1);
	CvMat *xhP = cvCreateMat(xh->cols, P->cols, CV_32FC1);
	CvMat *xhPyh = cvCreateMat(xh->cols, yh->cols, CV_32FC1);
	cvTranspose(xh, xhT);
	cvMatMul(xhT, P, xhP);
	cvMatMul(xhP, xh, xhPyh);	
	for (int i = 0; i < xh->rows; i++)
	{
		cvSetReal2D(xPy, i, 0, cvGetReal2D(xhPyh, i, i));
	}

	cvReleaseMat(&xhT);
	cvReleaseMat(&xhP);
	cvReleaseMat(&xhPyh);
}

void PrintMat(CvMat *M, string matrixName)
{
	cout << "Matrix: " << matrixName << endl;
	for (int i = 0 ; i < M->rows; i++)
	{
		for (int j = 0; j < M->cols; j++)
		{
			cout << cvGetReal2D(M, i, j) << " ";
		}
		cout << endl;
	}
}

void PrintMatRow(CvMat *M, int rowFrom, int rowTo, string matrixName)
{
	cout << "Matrix: " << matrixName << endl;
	for (int i = rowFrom ; i < min(rowTo+1, M->rows); i++)
	{
		for (int j = 0; j < M->cols; j++)
		{
			cout << cvGetReal2D(M, i, j) << " ";
		}
		cout << endl;
	}
}

void PrintMatCol(CvMat *M, int colFrom, int colTo, string matrixName)
{
	cout << "Matrix: " << matrixName << endl;
	for (int i = 0 ; i < M->rows; i++)
	{
		for (int j = colFrom; j < min(colTo+1, M->cols); j++)
		{
			cout << cvGetReal2D(M, i, j) << " ";
		}
		cout << endl;
	}
}

void GetSubMat(CvMat *M, int rowFrom, int rowTo, int colFrom, int colTo, CvMat *subM)
{
	for (int i = rowFrom; i < rowTo+1; i++)
	{
		for (int j = colFrom; j < colTo+1; j++)
			cvSetReal2D(subM, i-rowFrom, j-colFrom, cvGetReal2D(M, i, j));
	}
}

void SetSubMat(CvMat *M, int rowFrom, int rowTo, int colFrom, int colTo, CvMat *subM)
{
	for (int i = rowFrom; i < rowTo+1; i++)
	{
		for (int j = colFrom; j < colTo+1; j++)
			cvSetReal2D(M, i, j, cvGetReal2D(subM, i-rowFrom, j-colFrom));
	}
}

void SetSubMat(CvMat *M, int rowFrom, int colFrom, CvMat *subM)
{
	int rowTo = rowFrom + subM->rows;
	int colTo = colFrom + subM->cols;
	for (int i = rowFrom; i < rowTo; i++)
	{
		for (int j = colFrom; j < colTo; j++)
			cvSetReal2D(M, i, j, cvGetReal2D(subM, i-rowFrom, j-colFrom));
	}
}

void GetSubMatRowwise(CvMat *M, int rowFrom, int rowTo, CvMat *subM)
{
	for (int i = rowFrom; i < rowTo+1; i++)
	{
		for (int j = 0; j < M->cols; j++)
			cvSetReal2D(subM, i-rowFrom, j, cvGetReal2D(M, i, j));
	}
}

void GetSubMatColwise(CvMat *M, int colFrom, int colTo, CvMat *subM)
{
	for (int i = 0; i < M->rows; i++)
	{
		for (int j = colFrom; j < colTo+1; j++)
			cvSetReal2D(subM, i, j-colFrom, cvGetReal2D(M, i, j));
	}
}

void GetDCTMappingMatrix(CvMat *M, int n)
{
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			if (i == 0)
			{
				double Mij = sqrt(1.0/n)*cos(i*(2*j+1)*PI/2/n);
				cvSetReal2D(M, i, j, Mij);
			}
			else
			{
				double Mij = sqrt(2.0/n)*cos(i*(2*j+1)*PI/2/n);
				cvSetReal2D(M, i, j, Mij);
			}
		}
	}
}

void GetIDCTMappingMatrix(CvMat *M, int n)
{
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			if (j == 0)
			{
				double Mij = sqrt(1.0/n)*cos(j*(2*i+1)*PI/2/n);
				cvSetReal2D(M, i, j, Mij);
			}
			else
			{
				double Mij = sqrt(2.0/n)*cos(j*(2*i+1)*PI/2/n);
				cvSetReal2D(M, i, j, Mij);
			}
		}
	}
}

void DCTProjection(CvMat *P, CvMat *Theta, int nFrames, int iFrame, int nBase, CvMat *x)
{
	// P is camera matrix at i-th frame (3x4)
	// Theta is k-th 3D DCT coefficient vector (3nBasex1)
	// nFrames: the number of frames
	// iFrames: index for this frame (i-th frame)
	// nBase: the number of base
	// x: output (2D projection)
	CvMat *M = cvCreateMat(nFrames, nFrames, CV_32FC1);
	GetIDCTMappingMatrix(M, nFrames);
	CvMat *B = cvCreateMat(nFrames, nBase, CV_32FC1);
	// Truncating IDCT mapping matrix
	for (int i = 0; i < nFrames; i++)
	{
		for (int j = 0; j < nBase; j++)
		{
			cvSetReal2D(B, i, j, cvGetReal2D(M, i, j));
		}
	}
	// Create 1x(nBase) Bi^T matrix
	CvMat *BiT = cvCreateMat(1, nBase, CV_32FC1);
	// Create 3x(3nBase) BiTilde matrix
	CvMat *BiTilde = cvCreateMat(3, 3*nBase, CV_32FC1);
	// Extract i-th row of B matrix
	GetSubMatRowwise(B, iFrame, iFrame, BiT);
	//First row setting
	for (int i = 0; i < BiTilde->cols; i++)
	{
		if (i < nBase)
			cvSetReal2D(BiTilde, 0, i, cvGetReal2D(BiT, 0, i));
		else
			cvSetReal2D(BiTilde, 0, i, 0);
	}
	//Second row setting
	for (int i = 0; i < BiTilde->cols; i++)
	{
		if ((i >= nBase) && (i < 2*nBase))
			cvSetReal2D(BiTilde, 1, i, cvGetReal2D(BiT, 0, i-nBase));
		else
			cvSetReal2D(BiTilde, 1, i, 0);
	}
	//Third row setting
	for (int i = 0; i < BiTilde->cols; i++)
	{
		if (i >= 2*nBase)
			cvSetReal2D(BiTilde, 2, i, cvGetReal2D(BiT, 0, i-2*nBase));
		else
			cvSetReal2D(BiTilde, 2, i, 0);
	}

	CvMat *X = cvCreateMat(4,1, CV_32FC1); // homogeneous coordinate of 3D point
	CvMat *Xi = cvCreateMat(3,1, CV_32FC1); // inhomogeneous coordinate of 3D point

	// Xi = BiTilde*Theta ==> X = [BiTilde*Theta;1]
	cvMatMul(BiTilde, Theta, Xi);
	for (int i = 0; i < 3; i++)
	{
		cvSetReal2D(X, i, 0, cvGetReal2D(Xi, i, 0));
	}
	cvSetReal2D(X, 3, 0, 1);

	// Projection to 2D image
	CvMat *tempx = cvCreateMat(3,1, CV_32FC1);
	cvMatMul(P, X, tempx);
	cvSetReal2D(x, 0, 0, cvGetReal2D(tempx, 0, 0)/cvGetReal2D(tempx, 2, 0));
	cvSetReal2D(x, 1, 0, cvGetReal2D(tempx, 1, 0)/cvGetReal2D(tempx, 2, 0));

	cvReleaseMat(&M);
	cvReleaseMat(&B);
	cvReleaseMat(&BiT);
	cvReleaseMat(&BiTilde);
	cvReleaseMat(&X);
	cvReleaseMat(&Xi);
	cvReleaseMat(&tempx);
}

void GetBiTilde(int nFrames, int iFrame, int nBase, CvMat *BiTilde)
{
	CvMat *M = cvCreateMat(nFrames, nFrames, CV_32FC1);
	GetIDCTMappingMatrix(M, nFrames);
	CvMat *B = cvCreateMat(nFrames, nBase, CV_32FC1);
	// Truncating IDCT mapping matrix
	for (int i = 0; i < nFrames; i++)
	{
		for (int j = 0; j < nBase; j++)
		{
			cvSetReal2D(B, i, j, cvGetReal2D(M, i, j));
		}
	}
	// Create 1x(nBase) Bi^T matrix
	CvMat *BiT = cvCreateMat(1, nBase, CV_32FC1);
	// Extract i-th row of B matrix
	GetSubMatRowwise(B, iFrame, iFrame, BiT);
	//First row setting
	for (int i = 0; i < BiTilde->cols; i++)
	{
		if (i < nBase)
			cvSetReal2D(BiTilde, 0, i, cvGetReal2D(BiT, 0, i));
		else
			cvSetReal2D(BiTilde, 0, i, 0);
	}
	//Second row setting
	for (int i = 0; i < BiTilde->cols; i++)
	{
		if ((i >= nBase) && (i < 2*nBase))
			cvSetReal2D(BiTilde, 1, i, cvGetReal2D(BiT, 0, i-nBase));
		else
			cvSetReal2D(BiTilde, 1, i, 0);
	}
	//Third row setting
	for (int i = 0; i < BiTilde->cols; i++)
	{
		if (i >= 2*nBase)
			cvSetReal2D(BiTilde, 2, i, cvGetReal2D(BiT, 0, i-2*nBase));
		else
			cvSetReal2D(BiTilde, 2, i, 0);
	}
	cvReleaseMat(&M);
	cvReleaseMat(&B);
	cvReleaseMat(&BiT);
}

//void GetHugeB(int nFrames, int nBase, int nFeatures, CvSparseMat *hugeB)
//{
//	int sizeHugeBi[2] = {3*nFrames*nFeatures, 3*nFrames*nBase};
//	for (int iFrame = 0; iFrame < nFrames; iFrame++)
//	{
//		CvSparseMat *hugeBi = cvCreateSparseMat(2, sizeHugeBi, CV_32FC1);
//		CvMat *BiTilde = cvCreateMat(1, nBase, CV_32FC1);
//		GetBiTilde(nFrames, iFrame, nBase, BiTilde);
//		for (int i = 0; i < nFeatures; i++)
//			SetSubMat(hugeBi, 3*i, 3*nBase*i, BiTilde);
//		SetSubMat(hugeB, 3*nFeatures*i, 0, hugeBi);
//	}
//}

void GetHugeB(int nFrames, int nBase, int nFeatures, CvMat *hugeB)
{
	cvSetZero(hugeB);
	for (int iFrame = 0; iFrame < nFrames; iFrame++)
	{
		CvMat *hugeBi = cvCreateMat(3*nFeatures, 3*nFeatures*nBase, CV_32FC1);
		CvMat *BiTilde = cvCreateMat(3, 3*nBase, CV_32FC1);
		cvSetZero(hugeBi);
		cvSetZero(BiTilde);
		GetBiTilde(nFrames, iFrame, nBase, BiTilde);
		for (int i = 0; i < nFeatures; i++)
			SetSubMat(hugeBi, 3*i, 3*nBase*i, BiTilde);
		SetSubMat(hugeB, 3*nFeatures*iFrame, 0, hugeBi);
		cvReleaseMat(&hugeBi);
		cvReleaseMat(&BiTilde);
	}
}

//void GetHugeQ(int nFrames, int nFeatures, vector<Feature> vFeatures, vector<CvMat *> vCameraMatrix, CvSparseMat &hugeQ, CvMat &hugeq)
//{
//	vector<CvSparseMat *> vQiTilde;
//	vector<CvMat *> vqiTilde;
//	vector<int> vSizeQiTilde;
//	int nVisible = 0;
//	for (int iFrame = 0; iFrame < vCameraMatrix.size(); iFrame++)
//	{
//		vector<CvMat *> vQ, vq;
//		vector<int> visible;
//		for (int iFeature = 0; iFeature < vFeatures.size(); iFeature++)
//		{
//			vector<int>::const_iterator it = find(vFeatures[iFeature].vFrame.begin(),vFeatures[iFeature].vFrame.end(), iFrame);
//			if (it != vFeatures[iFeature].vFrame.end())
//			{
//				int idx = int(it-vFeatures[iFeature].vFrame.begin());
//				double u = vFeatures[iFeature].vx[idx];
//				double v = vFeatures[iFeature].vy[idx];
//				CvMat *P = cvCloneMat(vCameraMatrix[iFrame]);
//				double p11 = cvGetReal2D(P, 0,0);		double p12 = cvGetReal2D(P, 0,1);		double p13 = cvGetReal2D(P, 0,2);		double p14 = cvGetReal2D(P, 0,3);
//				double p21 = cvGetReal2D(P, 1,0);		double p22 = cvGetReal2D(P, 1,1);		double p23 = cvGetReal2D(P, 1,2);		double p24 = cvGetReal2D(P, 1,3);
//				double p31 = cvGetReal2D(P, 2,0);		double p32 = cvGetReal2D(P, 2,1);		double p33 = cvGetReal2D(P, 2,2);		double p34 = cvGetReal2D(P, 2,3);
//
//				double Q11 = v*p31-p21;		double Q12 = v*p32-p22;		double Q13 = v*p33-p23;
//				double Q21 = p11-u*p31;		double Q22 = p12-u*p32;		double Q23 = p13-u*p33;
//				double q1 = p24-v*p34;		double q2 = u*p34-p14;
//
//				CvMat *Qki = cvCreateMat(2,3, CV_32FC1);
//				CvMat *qki = cvCreateMat(2,1, CV_32FC1);
//
//				cvSetReal2D(Qki, 0, 0, Q11);		cvSetReal2D(Qki, 0, 1, Q12);		cvSetReal2D(Qki, 0, 2, Q13);
//				cvSetReal2D(Qki, 1, 0, Q21);		cvSetReal2D(Qki, 1, 1, Q22);		cvSetReal2D(Qki, 1, 2, Q23);
//
//				cvSetReal2D(qki, 0, 0, q1);			cvSetReal2D(qki, 1, 0, q2);
//				vQ.push_back(Qki);
//				vq.push_back(qki);
//				visible.push_back(iFeature);
//				nVisible++;
//			}
//		}
//
//		int sizeQiTilde[2] = {2*vQ.size(), 3*vQ.size()};
//		CvSparseMat *QiTilde = cvCreateSparseMat(2, sizeQiTilde, CV_32FC1);
//		CvMat *qiTilde = cvCreateMat(2*vQ.size(), 1, CV_32FC1);
//		for (int iQ = 0; iQ < vQ.size(); iQ++)
//		{
//			SetSubMat(QiTilde, 2*iQ, 3*visible[iQ], vQ[iQ]);
//			SetSubMat(qiTilde, 2*iQ, 0, vq[iQ]);
//		}
//		vQiTilde.push_back(QiTilde);
//		vqiTilde.push_back(qiTilde);
//		vSizeQiTilde.push_back(2*vQ.size());
//	}
//
//	int sizeQ[2] = {2*nVisible, 3*nVisible};
//	int last = 0;
//	&hugeQ = cvCreateSparseMat(2, sizeQ, CV_32FC1);
//	&hugeq = cvCreateMat(2*nVisible, 1, CV_32FC1);
//	for (int iQ = 0; iQ < vQiTilde.size(); iQ++)
//	{
//		SetSubMat(&hugeQ, last, 3*nFeatures*nFrames, vQiTilde[iQ]);
//		SetSubMat(&hugeq, last, 0, vqiTilde[iQ]);
//		last += vSizeQiTilde[iQ];
//	}
//}

void GetHugeQ(int nFrames, int nFeatures, vector<Feature> vFeatures, vector<CvMat *> vCameraMatrix, CvMat &hugeQ, CvMat &hugeq)
{
	vector<CvMat *> vQiTilde;
	vector<CvMat *> vqiTilde_q;
	vector<int> vSizeQiTilde;
	int nVisible = 0;
	for (int iFrame = 0; iFrame < vCameraMatrix.size(); iFrame++)
	{
		vector<CvMat*> vQ;
		vector<CvMat*> vq_q;
		vector<int> visible;
		for (int iFeature = 0; iFeature < vFeatures.size(); iFeature++)
		{
			vector<int>::const_iterator it = find(vFeatures[iFeature].vFrame.begin(),vFeatures[iFeature].vFrame.end(), iFrame);
			if (it != vFeatures[iFeature].vFrame.end())
			{
				int idx = int(it-vFeatures[iFeature].vFrame.begin());
				double u = vFeatures[iFeature].vx[idx];
				double v = vFeatures[iFeature].vy[idx];
				CvMat *P = cvCloneMat(vCameraMatrix[iFrame]);
			
				double p11 = cvGetReal2D(P, 0,0);		double p12 = cvGetReal2D(P, 0,1);		double p13 = cvGetReal2D(P, 0,2);		double p14 = cvGetReal2D(P, 0,3);
				double p21 = cvGetReal2D(P, 1,0);		double p22 = cvGetReal2D(P, 1,1);		double p23 = cvGetReal2D(P, 1,2);		double p24 = cvGetReal2D(P, 1,3);
				double p31 = cvGetReal2D(P, 2,0);		double p32 = cvGetReal2D(P, 2,1);		double p33 = cvGetReal2D(P, 2,2);		double p34 = cvGetReal2D(P, 2,3);

				double Q11 = v*p31-p21;		double Q12 = v*p32-p22;		double Q13 = v*p33-p23;
				double Q21 = p11-u*p31;		double Q22 = p12-u*p32;		double Q23 = p13-u*p33;
				double q1 = p24-v*p34;		double q2 = u*p34-p14;

				CvMat *Qki = cvCreateMat(2,3, CV_32FC1);
				CvMat *qki = cvCreateMat(2,1, CV_32FC1);

				cvSetReal2D(Qki, 0, 0, Q11);		cvSetReal2D(Qki, 0, 1, Q12);		cvSetReal2D(Qki, 0, 2, Q13);
				cvSetReal2D(Qki, 1, 0, Q21);		cvSetReal2D(Qki, 1, 1, Q22);		cvSetReal2D(Qki, 1, 2, Q23);

				cvSetReal2D(qki, 0, 0, q1);			cvSetReal2D(qki, 1, 0, q2);

				vQ.push_back(Qki);
				vq_q.push_back(qki);
				visible.push_back(iFeature);
				nVisible++;
				cvReleaseMat(&P);
			}
		}

		CvMat *QiTilde = cvCreateMat(2*vQ.size(), 3*nFeatures, CV_32FC1);
		CvMat *qiTilde = cvCreateMat(2*vQ.size(), 1, CV_32FC1);
		cvSetZero(QiTilde);
		cvSetZero(qiTilde);
		for (int iQ = 0; iQ < vQ.size(); iQ++)
		{
			SetSubMat(QiTilde, 2*iQ, 3*visible[iQ], vQ[iQ]);
			SetSubMat(qiTilde, 2*iQ, 0, vq_q[iQ]);
		}
		vQiTilde.push_back(QiTilde);
		vqiTilde_q.push_back(qiTilde);
		vSizeQiTilde.push_back(2*vQ.size());
		//cvReleaseMat(&QiTilde);
		//cvReleaseMat(&qiTilde);
	}

	int last = 0;
	hugeQ = *cvCreateMat(2*nVisible, 3*nFeatures*nFrames, CV_32FC1);
	hugeq = *cvCreateMat(2*nVisible, 1, CV_32FC1);
	cvSetZero(&hugeQ);
	cvSetZero(&hugeq);
	for (int iQ = 0; iQ < vQiTilde.size(); iQ++)
	{
		SetSubMat(&hugeQ, last, 3*iQ*nFeatures, vQiTilde[iQ]);
		SetSubMat(&hugeq, last, 0, vqiTilde_q[iQ]);
		last += vSizeQiTilde[iQ];
	}
}

void IDCT3D(CvMat *Theta, int nFrames, CvMat *X)
{
	int nBase = Theta->rows/3;
	CvMat *M = cvCreateMat(nFrames, nFrames, CV_32FC1);
	GetIDCTMappingMatrix(M, nFrames);
	CvMat *B = cvCreateMat(nFrames, nBase, CV_32FC1);
	// Truncating IDCT mapping matrix
	for (int i = 0; i < nFrames; i++)
	{
		for (int j = 0; j < nBase; j++)
		{
			cvSetReal2D(B, i, j, cvGetReal2D(M, i, j));
		}
	}
	CvMat *Xx = cvCreateMat(nFrames, 1, CV_32FC1);
	CvMat *Xy = cvCreateMat(nFrames, 1, CV_32FC1);
	CvMat *Xz = cvCreateMat(nFrames, 1, CV_32FC1);
	CvMat *Thetax = cvCreateMat(nBase, 1, CV_32FC1);
	CvMat *Thetay = cvCreateMat(nBase, 1, CV_32FC1);
	CvMat *Thetaz = cvCreateMat(nBase, 1, CV_32FC1);

	GetSubMat(Theta, 0, nBase-1, 0, 0, Thetax);
	GetSubMat(Theta, nBase, 2*nBase-1, 0, 0, Thetay);
	GetSubMat(Theta, 2*nBase, 3*nBase-1, 0, 0, Thetaz);

	cvMatMul(B, Thetax, Xx);
	cvMatMul(B, Thetay, Xy);
	cvMatMul(B, Thetaz, Xz);

	SetSubMat(X, 0, 0, Xx);
	SetSubMat(X, nFrames, 0, Thetax);
	SetSubMat(X, 2*nFrames, 0, Thetax);

	cvReleaseMat(&M);
	cvReleaseMat(&B);
	cvReleaseMat(&Xx);
	cvReleaseMat(&Xy);
	cvReleaseMat(&Xz);
	cvReleaseMat(&Thetax);
	cvReleaseMat(&Thetay);
	cvReleaseMat(&Thetaz);
}

void GetTheta(int nFrames, int nFeatures, int nBase, vector<Feature> vFeatures, vector<CvMat *> vCameraMatrix, CvMat &Theta)
{
	CvMat Theta1 = *cvCreateMat(3*nBase*nFeatures, 1, CV_32FC1);
	CvMat BB;
	GetBB(nFrames, nBase, BB);

	vector<int> badFeat;
	int nBad = 0;
	for (int iFeature = 0; iFeature < nFeatures; iFeature++)
	{
		CvMat Q;
		CvMat q;

		GetQ(nFrames, nBase, nFeatures, vCameraMatrix, vFeatures[iFeature], Q, q);
		CvMat *W = cvCreateMat(Q.cols, Q.cols, CV_32FC1);
		CvMat *wq = cvCreateMat(Q.rows+W->rows, 1, CV_32FC1);
		cvSetZero(wq);
		cvSetZero(W);
		SetSubMat(wq, 0, 0, &q);
		for (int i = 0; i < Q.cols/3; i++)
		{ 
			double weight = exp((double)(i+1)/2);
			cvSetReal2D(W, 3*i, 3*i, weight);
			cvSetReal2D(W, 3*i+1, 3*i+1, weight);
			cvSetReal2D(W, 3*i+2, 3*i+2, weight);
		}
				
		CvMat *QW = cvCreateMat(Q.rows+W->rows, Q.cols, CV_32FC1);

		SetSubMat(QW, 0, 0, &Q);
		SetSubMat(QW, Q.rows, 0, W);
		CvMat *QB = cvCreateMat(QW->rows, BB.cols, CV_32FC1);
		CvMat *QBinv = cvCreateMat(BB.cols, QW->rows, CV_32FC1);
		cvMatMul(QW, &BB, QB);
		cvInvert(QB, QBinv, CV_SVD);
		CvMat *U = cvCreateMat(QB->rows, QB->cols, CV_32FC1);
		CvMat *D = cvCreateMat(QB->cols, QB->cols, CV_32FC1);
		CvMat *V = cvCreateMat(QB->cols, QB->cols, CV_32FC1);
		cvSVD(QB, D, U, V);
		cout << cvGetReal2D(D, QB->cols-1, QB->cols-1) << endl;
		if (cvGetReal2D(D, QB->cols-1, QB->cols-1) < 0)
		{
			badFeat.push_back(iFeature);
			nBad++;
			continue;
		}
		else
			badFeat.push_back(-1);

		CvMat *Thetai = cvCreateMat(3*nBase, 1, CV_32FC1);
		cvMatMul(QBinv, wq, Thetai);

		SetSubMat(&Theta1, 3*iFeature*nBase, 0, Thetai);

		cvReleaseMat(&QB);
		cvReleaseMat(&QBinv);
		cvReleaseMat(&Thetai);
		cvReleaseMat(&U);
		cvReleaseMat(&D);
		cvReleaseMat(&W);
		cvReleaseMat(&V);
		cvReleaseMat(&QW);
		cvReleaseMat(&wq);
	}	
	Theta = *cvCreateMat(3*nBase*(nFeatures-nBad), 1, CV_32FC1);
	int last = 0;
	for (int iTh = 0; iTh < nFeatures; iTh++)
	{
		CvMat *Thetaj = cvCreateMat(3*nBase, 1, CV_32FC1);
		GetSubMat(&Theta1, 3*nBase*iTh, 3*nBase*iTh+3*nBase-1, 0, 0, Thetaj);
		if (badFeat[iTh] == -1)
		{
			SetSubMat(&Theta, last, 0, Thetaj);
			last += 3*nBase;
		}

		cvReleaseMat(&Thetaj);
	}
}

void GetThetaWOWeight(int nFrames, int nFeatures, int nBase, vector<Feature> vFeatures, vector<CvMat *> vCameraMatrix, CvMat &Theta)
{
	CvMat Theta1 = *cvCreateMat(3*nBase*nFeatures, 1, CV_32FC1);
	CvMat BB;
	GetBB(nFrames, nBase, BB);

	vector<int> badFeat;
	int nBad = 0;
	for (int iFeature = 0; iFeature < nFeatures; iFeature++)
	{
		CvMat Q;
		CvMat q;

		GetQ(nFrames, nBase, nFeatures, vCameraMatrix, vFeatures[iFeature], Q, q);
		CvMat *QB = cvCreateMat(Q.rows, BB.cols, CV_32FC1);
		CvMat *QBinv = cvCreateMat(BB.cols, Q.rows, CV_32FC1);
		cvMatMul(&Q, &BB, QB);
		cvInvert(QB, QBinv, CV_SVD);
		PrintMat(&Q);
		PrintMat(&q);
		PrintMat(QB);
		PrintMat(vCameraMatrix[2]);
		CvMat *U = cvCreateMat(QB->rows, QB->cols, CV_32FC1);
		CvMat *D = cvCreateMat(QB->cols, QB->cols, CV_32FC1);
		CvMat *V = cvCreateMat(QB->cols, QB->cols, CV_32FC1);
		cvSVD(QB, D, U, V);
		cout << cvGetReal2D(D, QB->cols-1, QB->cols-1) << endl;
		if (cvGetReal2D(D, QB->cols-1, QB->cols-1) < 0)
		{
			badFeat.push_back(iFeature);
			nBad++;
			continue;
		}
		else
			badFeat.push_back(-1);

		CvMat *Thetai = cvCreateMat(3*nBase, 1, CV_32FC1);
		cvMatMul(QBinv, &q, Thetai);

		SetSubMat(&Theta1, 3*iFeature*nBase, 0, Thetai);

		cvReleaseMat(&QB);
		cvReleaseMat(&QBinv);
		cvReleaseMat(&Thetai);
		cvReleaseMat(&U);
		cvReleaseMat(&D);
		cvReleaseMat(&V);

	}	
	Theta = *cvCreateMat(3*nBase*(nFeatures-nBad), 1, CV_32FC1);
	int last = 0;
	for (int iTh = 0; iTh < nFeatures; iTh++)
	{
		CvMat *Thetaj = cvCreateMat(3*nBase, 1, CV_32FC1);
		GetSubMat(&Theta1, 3*nBase*iTh, 3*nBase*iTh+3*nBase-1, 0, 0, Thetaj);
		if (badFeat[iTh] == -1)
		{
			SetSubMat(&Theta, last, 0, Thetaj);
			last += 3*nBase;
		}

		cvReleaseMat(&Thetaj);
	}
}

void GetThetaWOWeight(int nFrames, int nFeatures, int nBase, vector<Feature> vFeatures, vector<Camera> vCamera, CvMat &Theta)
{
	Theta = *cvCreateMat(3*nBase*nFeatures, 1, CV_32FC1);
	CvMat BB;
	GetBB(nFrames, nBase, BB);

	for (int iFeature = 0; iFeature < nFeatures; iFeature++)
	{
		CvMat Q;
		CvMat q;

		GetQ(nFrames, nBase, nFeatures, vCamera, vFeatures[iFeature], Q, q);
		//PrintMat(&Q);
		//PrintMat(&q);
		//PrintMat(&BB);
		CvMat *QB = cvCreateMat(Q.rows, BB.cols, CV_32FC1);
		CvMat *QBinv = cvCreateMat(BB.cols, Q.rows, CV_32FC1);
		cvMatMul(&Q, &BB, QB);
		cvInvert(QB, QBinv, CV_SVD);

		CvMat *Thetai = cvCreateMat(3*nBase, 1, CV_32FC1);
		cvMatMul(QBinv, &q, Thetai);

		SetSubMat(&Theta, 3*iFeature*nBase, 0, Thetai);
		//PrintMat(QB,"QB");
		//PrintMat(&q,"q");
		//PrintMat(Thetai);

		cvReleaseMat(&QB);
		cvReleaseMat(&QBinv);
		cvReleaseMat(&Thetai);
	}	
}

void GetThetaWOWeight(int max_nFrames, int nBase, vector<Feature> vFeatures, vector<Camera> vCamera, CvMat &Theta)
{
	Theta = *cvCreateMat(3*nBase*vFeatures.size(), 1, CV_32FC1);
	CvMat BB;
	GetBB(max_nFrames, nBase, BB);

	for (int iFeature = 0; iFeature < vFeatures.size(); iFeature++)
	{
		CvMat Q;
		CvMat q;

		GetQ(max_nFrames, nBase, vFeatures.size(), vCamera, vFeatures[iFeature], Q, q);
		CvMat *QB = cvCreateMat(Q.rows, BB.cols, CV_32FC1);
		CvMat *QBinv = cvCreateMat(BB.cols, Q.rows, CV_32FC1);
		cvMatMul(&Q, &BB, QB);
		cvInvert(QB, QBinv, CV_SVD);

		CvMat *Thetai = cvCreateMat(3*nBase, 1, CV_32FC1);
		cvMatMul(QBinv, &q, Thetai);

		SetSubMat(&Theta, 3*iFeature*nBase, 0, Thetai);
		//PrintMat(QB,"QB");
		//PrintMat(&Q,"Q");
		//PrintMat(&q,"q");
		//PrintMat(Thetai, "Thetai");

		cvReleaseMat(&QB);
		cvReleaseMat(&QBinv);
		cvReleaseMat(&Thetai);
	}	
}

void GetThetaWOWeight_basis(int max_nFrames, vector<int> vnBase, vector<Feature> vFeatures, vector<Camera> vCamera, vector<CvMat *> &vTheta)
{
	for (int iFeature = 0; iFeature < vFeatures.size(); iFeature++)
	{
		CvMat BB;
		GetBB(max_nFrames, vnBase[iFeature], BB);

		CvMat Q;
		CvMat q;

		GetQ(max_nFrames, vnBase[iFeature], vFeatures.size(), vCamera, vFeatures[iFeature], Q, q);
		CvMat *QB = cvCreateMat(Q.rows, BB.cols, CV_32FC1);
		CvMat *QBinv = cvCreateMat(BB.cols, Q.rows, CV_32FC1);
		cvMatMul(&Q, &BB, QB);
		cvInvert(QB, QBinv, CV_SVD);

		CvMat *Thetai = cvCreateMat(3*vnBase[iFeature], 1, CV_32FC1);
		cvMatMul(QBinv, &q, Thetai);

		//PrintMat(QB,"QB");
		//PrintMat(&Q,"Q");
		//PrintMat(&q,"q");
		//PrintMat(Thetai, "Thetai");

		vTheta.push_back(Thetai);

		cvReleaseMat(&QB);
		cvReleaseMat(&QBinv);
	}	
}

void GetThetaWOWeight_OrthogonalToCamera(int max_nFrames, int nBase, vector<Feature> vFeatures, vector<Camera> vCamera, CvMat &Theta, CvMat &BaseB, CvMat &CameraTraj, bool mode)
{
	Theta = *cvCreateMat(3*nBase*vFeatures.size(), 1, CV_32FC1);

	//CvMat CameraMatrix;
	////vCamera[0].vTakenFrame.erase(vCamera[0].vTakenFrame.begin()+4, vCamera[0].vTakenFrame.end());
	////vCamera[1].vTakenFrame.erase(vCamera[1].vTakenFrame.begin()+4, vCamera[1].vTakenFrame.end());
	////vCamera[2].vTakenFrame.erase(vCamera[2].vTakenFrame.begin()+4, vCamera[2].vTakenFrame.end());
	////vCamera[3].vTakenFrame.erase(vCamera[3].vTakenFrame.begin()+4, vCamera[3].vTakenFrame.end());
	//CreatCameraMatrix(vCamera, max_nFrames, CameraMatrix);

	vector<double> vcx1, vcy1, vcz1;
	vector<double> vcx2, vcy2, vcz2;
	for (int iFrame = 0; iFrame < max_nFrames; iFrame++)
	{
		vector<double> vx, vy, vz;
		for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
		{
			//if ((iCamera == 1) || (iCamera == 2)/* || (iCamera == 3)*/)
			//	continue;
			vector<int>::const_iterator it = find(vCamera[iCamera].vTakenFrame.begin(),vCamera[iCamera].vTakenFrame.end(), iFrame);
			if (it != vCamera[iCamera].vTakenFrame.end())
			{
				int idx = int(it-vCamera[iCamera].vTakenFrame.begin());
				CvMat *P = cvCloneMat(vCamera[iCamera].vP[idx]);
				CvMat *K = cvCloneMat(vCamera[iCamera].vK[idx]);
				CvMat *C = cvCreateMat(3,1,CV_32FC1);
				CvMat *R = cvCreateMat(3,3,CV_32FC1);
				GetCameraParameter(P, K, R, C);
				vx.push_back(cvGetReal2D(C, 0, 0));
				vy.push_back(cvGetReal2D(C, 1, 0));
				vz.push_back(cvGetReal2D(C, 2, 0));
			}
		}
		if (vcx1.size() == 0)
		{
			vcx1.push_back(vx[0]);
			vcy1.push_back(vy[0]);
			vcz1.push_back(vz[0]);

			vcx2.push_back(vx[0]);
			vcy2.push_back(vy[0]);
			vcz2.push_back(vz[0]);
		}
		else if (vx.size() == 0)
		{
			vcx1.push_back(vcx1[vcx1.size()-1]);
			vcy1.push_back(vcy1[vcy1.size()-1]);
			vcz1.push_back(vcz1[vcz1.size()-1]);

			vcx2.push_back(vcx2[vcx2.size()-1]);
			vcy2.push_back(vcy2[vcy2.size()-1]);
			vcz2.push_back(vcz2[vcz2.size()-1]);
		}
		else
		{
			vcx2.push_back(vx[0]);
			vcy2.push_back(vy[0]);
			vcz2.push_back(vz[0]);

			double max_dist = 0;
			int idx = 0;
			for (int ix = 0; ix < vx.size(); ix++)
			{
				double dist = (vcx1[vcx1.size()-1]-vx[ix])*(vcx1[vcx1.size()-1]-vx[ix]) 
							+ (vcy1[vcy1.size()-1]-vy[ix])*(vcy1[vcy1.size()-1]-vy[ix]) 
							+ (vcz1[vcz1.size()-1]-vz[ix])*(vcz1[vcz1.size()-1]-vz[ix]);
				if (dist > max_dist)
				{
					max_dist = dist;
					idx = ix;
				}
			}
			vcx1.push_back(vx[idx]);
			vcy1.push_back(vy[idx]);
			vcz1.push_back(vz[idx]);
		}
	}

	//vcx2.clear();
	//vcy2.clear();
	//vcz2.clear();

	//int cameraIndex = 0;
	//for (int iFrame = 0; iFrame < max_nFrames; iFrame++)
	//{
	//	vector<int>::const_iterator it = find(vCamera[cameraIndex].vTakenFrame.begin(), vCamera[cameraIndex].vTakenFrame.end(), iFrame);
	//	if (it != vCamera[cameraIndex].vTakenFrame.end())
	//	{
	//		int idx = int(it-vCamera[cameraIndex].vTakenFrame.begin());
	//		CvMat *P = cvCloneMat(vCamera[cameraIndex].vP[idx]);
	//		CvMat *K = cvCloneMat(vCamera[cameraIndex].vK[idx]);
	//		CvMat *C = cvCreateMat(3,1,CV_32FC1);
	//		CvMat *R = cvCreateMat(3,3,CV_32FC1);
	//		GetCameraParameter(P, K, R, C);

	//		vcx2.push_back(cvGetReal2D(C, 0, 0));
	//		vcy2.push_back(cvGetReal2D(C, 1, 0));
	//		vcz2.push_back(cvGetReal2D(C, 2, 0));
	//	}
	//	else
	//	{
	//		vcx2.push_back(vcx2[vcx2.size()-1]);
	//		vcy2.push_back(vcy2[vcy2.size()-1]);
	//		vcz2.push_back(vcz2[vcz2.size()-1]);
	//	}
	//}


	//int nTotFrames = 0;
	//for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	//{
	//	nTotFrames += vCamera[iCamera].vTakenFrame.size();
	//}

	//vector<double> vcx1, vcy1, vcz1;
	//vector<double> vcx1, vcy1, vcz1;
	//for (int iCameraSpace = 0; iCameraSpace < 4; iCameraSpace++)
	//{
	//	vector<double> vx, vy, vz;
	//	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	//	{
	//		vector<int>::const_iterator it = find(vCamera[iCamera].vTakenFrame.begin(),vCamera[iCamera].vTakenFrame.end(), iFrame);
	//		if (it != vCamera[iCamera].vTakenFrame.end())
	//		{
	//			int idx = int(it-vCamera[iCamera].vTakenFrame.begin());
	//			CvMat *P = cvCloneMat(vCamera[iCamera].vP[idx]);
	//			CvMat *K = cvCloneMat(vCamera[iCamera].vK[idx]);
	//			CvMat *C = cvCreateMat(3,1,CV_32FC1);
	//			CvMat *R = cvCreateMat(3,3,CV_32FC1);
	//			GetCameraParameter(P, K, R, C);
	//			vx.push_back(cvGetReal2D(C, 0, 0));
	//			vy.push_back(cvGetReal2D(C, 1, 0));
	//			vz.push_back(cvGetReal2D(C, 2, 0));
	//		}
	//	}

	//	if (vx.size() == 0)
	//	{
	//		vcx.push_back(vcx[vcx.size()-1]);
	//		vcx.push_back(vcx[vcx.size()-1]);
	//		vcx.push_back(vcx[vcx.size()-1]);
	//	}
	//	
	//}

	CvMat *CameraTrajectory;
	CvMat CameraTrajectoryTr;
	if (!mode)
	{
		CameraTrajectory = cvCreateMat(2, 3*max_nFrames, CV_32FC1);
		cvSetZero(CameraTrajectory);
		for (int iFrame = 0; iFrame < max_nFrames; iFrame++)
		{
			cvSetReal2D(CameraTrajectory, 0, 3*iFrame, vcx1[iFrame]);
			cvSetReal2D(CameraTrajectory, 0, 3*iFrame+1, vcy1[iFrame]);
			cvSetReal2D(CameraTrajectory, 0, 3*iFrame+2, vcz1[iFrame]);

			cvSetReal2D(CameraTrajectory, 1, 3*iFrame, vcx2[iFrame]);
			cvSetReal2D(CameraTrajectory, 1, 3*iFrame+1, vcy2[iFrame]);
			cvSetReal2D(CameraTrajectory, 1, 3*iFrame+2, vcz2[iFrame]);
		}
		CameraTrajectoryTr = *cvCreateMat(3*max_nFrames, 2, CV_32FC1);
	}
	else
	{
		//vcx1 = vcx2;
		//vcy1 = vcy2;
		//vcz1 = vcz2;
		CameraTrajectory = cvCreateMat(1, 3*max_nFrames, CV_32FC1);
		cvSetZero(CameraTrajectory);
		for (int iFrame = 0; iFrame < max_nFrames; iFrame++)
		{
			cvSetReal2D(CameraTrajectory, 0, 3*iFrame, vcx1[iFrame]);
			cvSetReal2D(CameraTrajectory, 0, 3*iFrame+1, vcy1[iFrame]);
			cvSetReal2D(CameraTrajectory, 0, 3*iFrame+2, vcz1[iFrame]);
		}
		CameraTrajectoryTr = *cvCreateMat(3*max_nFrames, 1, CV_32FC1);
	}

	cvTranspose(CameraTrajectory, &CameraTrajectoryTr);

	CvMat B1, B2;
	GetB12(max_nFrames, nBase, B1, B2);

	CvMat *B_ = cvCreateMat(3*max_nFrames, B1.cols+B2.cols+CameraTrajectoryTr.cols, CV_32FC1);
	CvMat *B_ortho = cvCreateMat(3*max_nFrames, B1.cols+B2.cols+CameraTrajectoryTr.cols, CV_32FC1);
	cvSetZero(B_);
	cvSetZero(B_ortho);
	SetSubMat(B_, 0, 0, &B1);
	SetSubMat(B_, 0, 3, &CameraTrajectoryTr);
	SetSubMat(B_, 0, 3+CameraTrajectoryTr.cols, &B2);
	MatrixOrthogonalization(B_, B_ortho);
	CvMat BB = *cvCreateMat(3*max_nFrames, B1.cols+B2.cols, CV_32FC1);
	cvSetZero(&BB);
	SetSubMat(&BB, 0, 0, &B1);
	GetSubMatColwise(B_ortho, B1.cols+CameraTrajectoryTr.cols, B1.cols+B2.cols+CameraTrajectoryTr.cols-1, &B2);
	//GetSubMatColwise(B_ortho, B1.cols, B1.cols+CameraTrajectoryTr.cols-1, &CameraTrajectoryTr);
	SetSubMat(&BB, 0, 3, &B2);

	CvMat *CB = cvCreateMat(CameraTrajectoryTr.cols, BB.cols, CV_32FC1);
	CvMat *ct = cvCreateMat(CameraTrajectoryTr.cols, CameraTrajectoryTr.rows, CV_32FC1);
	cvTranspose(&CameraTrajectoryTr, ct);
	cvMatMul(ct, &BB, CB);
	PrintMat(CB);





	//CvMat *M = cvCreateMat(max_nFrames, max_nFrames, CV_32FC1);
	//GetIDCTMappingMatrix(M, max_nFrames);
	//vector<CvMat *> vBB;
	//for (int iFrame = 0; iFrame < max_nFrames; iFrame++)
	//{
	//	CvMat *B = cvCreateMat(1, nBase, CV_32FC1);
	//	GetSubMat(M, iFrame, iFrame, 0, nBase-1, B);
	//	vector<CvMat *> vC;
	//	CvMat *Bi = cvCreateMat(3, 3*nBase, CV_32FC1);

	//	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	//	{
	//		vector<int>::const_iterator it = find(vCamera[iCamera].vTakenFrame.begin(),vCamera[iCamera].vTakenFrame.end(), iFrame);
	//		if (it != vCamera[iCamera].vTakenFrame.end())
	//		{
	//			int idx = int(it-vCamera[iCamera].vTakenFrame.begin());
	//			CvMat *P = cvCloneMat(vCamera[iCamera].vP[idx]);
	//			CvMat *K = cvCloneMat(vCamera[iCamera].vK[idx]);
	//			CvMat *C = cvCreateMat(3,1,CV_32FC1);
	//			CvMat *R = cvCreateMat(3,3,CV_32FC1);
	//			GetCameraParameter(P, K, R, C);
	//			vC.push_back(C);
	//		}
	//	}
	//	GetBi_ortho(vC, B, Bi);		
	//	vBB.push_back(Bi);
	//}
	
	//CvMat BB = *cvCreateMat(3*max_nFrames, 3*nBase, CV_32FC1);
	//for (int iBi = 0; iBi < vBB.size(); iBi++)
	//{
	//	SetSubMat(&BB, 3*iBi, 0, vBB[iBi]);
	//}

	

	//CvMat BB;
	//GetBB(max_nFrames, nBase, BB);

	//CvMat *BB_ = cvCreateMat(BB.rows, BB.cols+1, CV_32FC1);
	//cvSetZero(BB_);
	//CvMat *b = cvCreateMat(BB.rows, 1, CV_32FC1);
	//CvMat *b3 = cvCreateMat(BB.rows, 3, CV_32FC1);

	//GetSubMatColwise(&BB, 0, 0, b);
	//SetSubMat(BB_, 0, 0, b);
	//GetSubMatColwise(&BB, nBase, nBase, b);
	//SetSubMat(BB_, 0, 1, b);
	//GetSubMatColwise(&BB, 2*nBase, 2*nBase, b);
	//SetSubMat(BB_, 0, 2, b);

	//SetSubMat(BB_, 0, 3, CameraTrajectoryTr);
	//for (int icol = 1; icol < nBase; icol++)
	//{
	//	GetSubMatColwise(&BB, icol, icol, b);
	//	SetSubMat(BB_, 0, 3*icol+1, b);
	//	GetSubMatColwise(&BB, icol+nBase, icol+nBase, b);
	//	SetSubMat(BB_, 0, 3*icol+2, b);
	//	GetSubMatColwise(&BB, icol+2*nBase, icol+2*nBase, b);
	//	SetSubMat(BB_, 0, 3*icol+3, b);
	//}
	//CvMat *BB_ortho = cvCloneMat(BB_);
	//cvSetZero(BB_ortho);

	////PrintMat(BB_);
	//MatrixOrthogonalization(BB_, BB_ortho);

	//GetSubMatColwise(BB_ortho, 0, 2, b3);
	//SetSubMat(&BB, 0, 0, b3);

	//CvMat *CB = cvCreateMat(1, BB.cols, CV_32FC1);
	//cvMatMul(CameraTrajectory, &BB, CB);
	//PrintMat(CB);

	//for (int icol = 4; icol < BB_ortho->cols; icol++)
	//{
	//	GetSubMatColwise(BB_ortho, icol, icol, b);
	//	SetSubMat(&BB, 0, icol-1, b);
	//}
	//

	CvMat BB_pure, BB_pure_tr;
	GetBB(max_nFrames, nBase, BB_pure);
	BB_pure_tr = *cvCreateMat(BB_pure.cols, BB_pure.rows, CV_32FC1);
	cvTranspose(&BB_pure, &BB_pure_tr);
	CvMat *BB_temp = cvCreateMat(BB_pure_tr.rows, BB.cols, CV_32FC1);
	CvMat *BB_temp1 = cvCreateMat(BB_pure_tr.rows, BB_pure.cols, CV_32FC1);

	CvMat Bt = *cvCloneMat(&BB);
	cvMatMul(&BB_pure_tr, &BB_pure, BB_temp1);
	//PrintMat(BB_temp1);
	cvMatMul(&BB_pure_tr, &Bt, BB_temp);
	cvMatMul(&BB_pure, BB_temp, &BB);
	//PrintMat(&BB);
	//PrintMat(&BB_pure_tr);
	for (int iFeature = 0; iFeature < vFeatures.size(); iFeature++)
	{
		CvMat Q;
		CvMat q;

		GetQ(max_nFrames, nBase, vFeatures.size(), vCamera, vFeatures[iFeature], Q, q);
		CvMat *QB = cvCreateMat(Q.rows, BB.cols, CV_32FC1);
		CvMat *QBinv = cvCreateMat(BB.cols, Q.rows, CV_32FC1);
		cvMatMul(&Q, &BB, QB);
		cvInvert(QB, QBinv, CV_SVD);

		CvMat *Thetai = cvCreateMat(3*nBase, 1, CV_32FC1);
		cvMatMul(QBinv, &q, Thetai);

		CvMat *X = cvCreateMat(BB.rows, 1, CV_32FC1);
		cvMatMul(&BB, Thetai, X);		
		//PrintMat(X);
		cvMatMul(&BB_pure_tr, X, Thetai);
		//PrintMat(Thetai);

		SetSubMat(&Theta, 3*iFeature*nBase, 0, Thetai);
		//PrintMat(QB,"QB");
		//PrintMat(&Q,"Q");
		//PrintMat(&q,"q");
		//PrintMat(Thetai, "Thetai");

		cvReleaseMat(&QB);
		cvReleaseMat(&QBinv);
		cvReleaseMat(&Thetai);
	}	

	//CvMat *BB_pure_BB = cvCreateMat(BB_pure_tr.rows, BB.cols, CV_32FC1);
	//cvMatMul(&BB_pure_tr, &BB, BB_pure_BB);
	//cvMatMul(&BB_pure, BB_pure_BB, &BB);
	BaseB = *cvCloneMat(&BB);
	CameraTraj = *cvCloneMat(CameraTrajectory);

	cvReleaseMat(&CameraTrajectory);
	//cvReleaseMat(&CameraTrajectoryTr);
	//cvReleaseMat(&BB_);
	//cvReleaseMat(&b);
	//cvReleaseMat(&b3);
	//cvReleaseMat(&BB_ortho);
}

void GetBi_ortho(vector<CvMat*> vC, CvMat *B, CvMat *Bi)
{
	cvSetZero(Bi);
	//SetSubMat(Bi, 0, 0, B);
	//SetSubMat(Bi, 1, B->cols, B);
	//SetSubMat(Bi, 1, 2*B->cols, B);
	CvMat *B_ = cvCreateMat(3, vC.size()+Bi->cols, CV_32FC1);
	CvMat *B_ortho = cvCreateMat(3, vC.size()+Bi->cols, CV_32FC1);
	cvSetZero(B_);
	cvSetZero(B_ortho);

	cvSetReal2D(Bi, 0, 0, cvGetReal2D(B, 0, 0));
	cvSetReal2D(Bi, 1, 1, cvGetReal2D(B, 0, 0));
	cvSetReal2D(Bi, 2, 2, cvGetReal2D(B, 0, 0));

	for (int iC = 0; iC < vC.size(); iC++)
	{
		SetSubMat(Bi, 0, 3+iC, vC[iC]);
	}

	for (int iB = 0; iB < B->cols-1; iB++)
	{
		cvSetReal2D(Bi, 0, 3+vC.size()+3*iB, cvGetReal2D(B, 0, iB+1));
		cvSetReal2D(Bi, 1, 3+vC.size()+3*iB+1, cvGetReal2D(B, 0, iB+1));
		cvSetReal2D(Bi, 2, 3+vC.size()+3*iB+2, cvGetReal2D(B, 0, iB+1));
	}

	//SetSubMat(B_, 0, 3+vC.size()-1, Bi);
	MatrixOrthogonalization(B_, B_ortho);

	GetSubMatColwise(B_ortho, 3+vC.size()-1, B_ortho->cols-1, Bi);
	PrintMat(B_ortho,"b");
}

void CreatCameraMatrix(vector<Camera> vCamera, int max_nFrame, CvMat &CameraMatrix)
{
	int nFrameTot = 0;
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		nFrameTot += vCamera[iCamera].vTakenFrame.size();
	}
	CameraMatrix = *cvCreateMat(3*max_nFrame, nFrameTot, CV_32FC1);
	cvSetZero(&CameraMatrix);
	int beginIdx = 0;
	for (int iFrame = 0; iFrame < max_nFrame; iFrame++)
	{
		for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
		{
			vector<int>::const_iterator it = find(vCamera[iCamera].vTakenFrame.begin(),vCamera[iCamera].vTakenFrame.end(), iFrame);
			if (it != vCamera[iCamera].vTakenFrame.end())
			{
				int idx = int(it-vCamera[iCamera].vTakenFrame.begin());
				CvMat *P = cvCloneMat(vCamera[iCamera].vP[idx]);
				CvMat *K = cvCloneMat(vCamera[iCamera].vK[idx]);
				CvMat *C = cvCreateMat(3,1,CV_32FC1);
				CvMat *R = cvCreateMat(3,3,CV_32FC1);
				GetCameraParameter(P, K, R, C);
				SetSubMat(&CameraMatrix, 3*vCamera[iCamera].vTakenFrame[idx], beginIdx, C);
				beginIdx++;

			}
		}
	}
}

void GetB12(int max_nFrame, int nBase, CvMat &B1, CvMat &B2)
{
	CvMat *M = cvCreateMat(max_nFrame, max_nFrame, CV_32FC1);
	GetIDCTMappingMatrix(M, max_nFrame);
	B1 = *cvCreateMat(3*max_nFrame, 3, CV_32FC1);
	cvSetZero(&B1);
	for (int iFrame = 0; iFrame < max_nFrame; iFrame++)
	{
		cvSetReal2D(&B1, 3*iFrame, 0, cvGetReal2D(M, iFrame, 0));
		cvSetReal2D(&B1, 3*iFrame+1, 1, cvGetReal2D(M, iFrame, 0));
		cvSetReal2D(&B1, 3*iFrame+2, 2, cvGetReal2D(M, iFrame, 0));
	}
	B2 = *cvCreateMat(3*max_nFrame, 3*(nBase-1), CV_32FC1);
	cvSetZero(&B2);
	for (int iFrame = 0; iFrame < max_nFrame; iFrame++)
	{
		for (int iBase = 1; iBase < nBase; iBase++)
		{
			cvSetReal2D(&B2, 3*iFrame, 3*(iBase-1), cvGetReal2D(M, iFrame, iBase));
			cvSetReal2D(&B2, 3*iFrame+1, 3*(iBase-1)+1, cvGetReal2D(M, iFrame, iBase));
			cvSetReal2D(&B2, 3*iFrame+2, 3*(iBase-1)+2, cvGetReal2D(M, iFrame, iBase));
		}
	}
}

void GetQ(int nFrames, int nBase, int nFeatures, vector<CvMat *> vCameraMatrix, Feature feature, CvMat &Q, CvMat &q)
{
	CvMat *x0 = cvCreateMat(feature.vFrame.size(), 2, CV_32FC1);
	CvMat *x1 = cvCreateMat(feature.vFrame.size(), 2, CV_32FC1);
	CvMat *T = cvCreateMat(3, 3, CV_32FC1);
	cvSetZero(x1);	cvSetZero(T);
	for (int iFeature = 0; iFeature < feature.vFrame.size(); iFeature++)
	{
		cvSetReal2D(x0, iFeature, 0, feature.vx[iFeature]);
		cvSetReal2D(x0, iFeature, 1, feature.vy[iFeature]);
	}
	Normalization(x0, x1, T);
	x1 = cvCloneMat(x0);
	cvSetIdentity(T);
	
	Q = *cvCreateMat(2*feature.vFrame.size(), 3*nFrames, CV_32FC1);
	q = *cvCreateMat(2*feature.vFrame.size(), 1, CV_32FC1);
	cvSetZero(&Q);	cvSetZero(&q);
	int last = 0;
	for (int iFrame = 0; iFrame < nFrames; iFrame++)
	{
		vector<int>::const_iterator it = find(feature.vFrame.begin(),feature.vFrame.end(), iFrame);
		if (it != feature.vFrame.end())
		{
			int idx = int(it-feature.vFrame.begin());
			CvMat *TP = cvCreateMat(3, 4, CV_32FC1);
			CvMat *P = cvCloneMat(vCameraMatrix[iFrame]);
			cvMatMul(T, P, TP);
			double u = cvGetReal2D(x1, idx, 0);
			double v = cvGetReal2D(x1, idx, 1);
			double p11 = cvGetReal2D(TP, 0,0);		double p12 = cvGetReal2D(TP, 0,1);		double p13 = cvGetReal2D(TP, 0,2);		double p14 = cvGetReal2D(TP, 0,3);
			double p21 = cvGetReal2D(TP, 1,0);		double p22 = cvGetReal2D(TP, 1,1);		double p23 = cvGetReal2D(TP, 1,2);		double p24 = cvGetReal2D(TP, 1,3);
			double p31 = cvGetReal2D(TP, 2,0);		double p32 = cvGetReal2D(TP, 2,1);		double p33 = cvGetReal2D(TP, 2,2);		double p34 = cvGetReal2D(TP, 2,3);

			double Q11 = v*p31-p21;		double Q12 = v*p32-p22;		double Q13 = v*p33-p23;
			double Q21 = p11-u*p31;		double Q22 = p12-u*p32;		double Q23 = p13-u*p33;
			double q1 = p24-v*p34;		double q2 = u*p34-p14;

			CvMat *Qi = cvCreateMat(2,3, CV_32FC1);
			CvMat *qi = cvCreateMat(2,1, CV_32FC1);
			cvSetReal2D(Qi, 0, 0, Q11);		cvSetReal2D(Qi, 0, 1, Q12);		cvSetReal2D(Qi, 0, 2, Q13);
			cvSetReal2D(Qi, 1, 0, Q21);		cvSetReal2D(Qi, 1, 1, Q22);		cvSetReal2D(Qi, 1, 2, Q23);
			cvSetReal2D(qi, 0, 0, q1);		cvSetReal2D(qi, 1, 0, q2);
			SetSubMat(&Q, 2*last, 3*iFrame, Qi);
			SetSubMat(&q, 2*last, 0, qi);
			last++;
			cvReleaseMat(&TP);
			cvReleaseMat(&P);
			cvReleaseMat(&Qi);
			cvReleaseMat(&qi);
		}
	}
	cvReleaseMat(&x0);
	cvReleaseMat(&x1);
	cvReleaseMat(&T);
}


void GetQ(int nFrames, int nBase, int nFeatures, vector<Camera> vCamera, Feature feature, CvMat &Q, CvMat &q)
{
	int last = 0;
	Q = *cvCreateMat(2*feature.vFrame.size(), 3*nFrames, CV_32FC1);
	q = *cvCreateMat(2*feature.vFrame.size(), 1, CV_32FC1);
	cvSetZero(&Q);	cvSetZero(&q);
	
	for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	{
		//if ((iCamera == 1) /*|| (iCamera == 2) || (iCamera == 3)*/)
		//	continue;
		vector<double> vx0, vy0;
		vector<int> vxInd;
		for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
		{
			int frame = iCamera*nFrames + vCamera[iCamera].vTakenFrame[iFrame];
			vector<int>::const_iterator it = find(feature.vFrame.begin(),feature.vFrame.end(), frame);
			if (it != feature.vFrame.end())
			{
				int idx = (int) (it - feature.vFrame.begin());
				vx0.push_back(feature.vx[idx]);
				vy0.push_back(feature.vy[idx]);
				//vxInd.push_back(feature.vFrame[idx]);
				vxInd.push_back(frame);
			}
		}
		if (vx0.size() == 0)
			continue;
		CvMat *x0 = cvCreateMat(vx0.size(), 2, CV_32FC1);
		CvMat *x1 = cvCreateMat(vx0.size(), 2, CV_32FC1);
		CvMat *T = cvCreateMat(3, 3, CV_32FC1);
		cvSetZero(x1);	cvSetZero(T);
		for (int iFeature = 0; iFeature < vx0.size(); iFeature++)
		{
			cvSetReal2D(x0, iFeature, 0, vx0[iFeature]);
			cvSetReal2D(x0, iFeature, 1, vy0[iFeature]);
		}
		Normalization(x0, x1, T);
		x1 = cvCloneMat(x0);
		cvSetIdentity(T);

		for (int iFeatureFrame = 0; iFeatureFrame < vxInd.size(); iFeatureFrame++)
		{
			int iCamera = (int) (double)vxInd[iFeatureFrame]/nFrames;
			int iFrame = vxInd[iFeatureFrame] % nFrames;

			CvMat *P = cvCreateMat(3,4,CV_32FC1);
			CvMat *TP = cvCreateMat(3, 4, CV_32FC1);
			vector<int>::const_iterator it = find(vCamera[iCamera].vTakenFrame.begin(), vCamera[iCamera].vTakenFrame.end(), iFrame);
			int iP = (int) (it - vCamera[iCamera].vTakenFrame.begin());
			P = cvCloneMat(vCamera[iCamera].vP[iP]);

			CvMat *K = cvCreateMat(3,3, CV_32FC1);
			K = cvCloneMat(vCamera[iCamera].vK[iP]);
			CvMat *R = cvCreateMat(3,3, CV_32FC1);
			CvMat *C = cvCreateMat(3,1, CV_32FC1);
			CvMat *invK = cvCreateMat(3,3,CV_32FC1);
			CvMat *invR = cvCreateMat(3,3,CV_32FC1);
			CvMat *invRinvK = cvCreateMat(3,3, CV_32FC1);
			CvMat *invx = cvCreateMat(3,1, CV_32FC1);
			CvMat *x = cvCreateMat(3,1, CV_32FC1);

			double u = cvGetReal2D(x1, iFeatureFrame, 0);
			double v = cvGetReal2D(x1, iFeatureFrame, 1);
			cvSetZero(x);
			cvSetReal2D(x, 0, 0, u);
			cvSetReal2D(x, 1, 0, v);
			cvSetReal2D(x, 2, 0, 1);
		
			GetCameraParameter(P, K, R, C);
			//PrintMat(P);
			ScalarMul(C, -1, C);
			
			
			cvInvert(K, invK);
			cvTranspose(R, invR);
			cvMatMul(invR, invK, invRinvK);
			//PrintMat(invR);
			//PrintMat(invK);
			cvMatMul(invRinvK, x, invx);
			//PrintMat(invx);
			//u = cvGetReal2D(invx, 0, 0)/cvGetReal2D(invx, 2, 0);
			//v = cvGetReal2D(invx, 1, 0)/cvGetReal2D(invx, 2, 0);
			////cvSetIdentity(P);
			////SetSubMat(P, 0, 3, C);
			//cvMatMul(invRinvK, P, P);

			//PrintMat(P);


			//PrintMat(T);
			cvMatMul(T, P, TP);
			//if ((cvGetReal2D(TP, 0, 0) == 1) && (cvGetReal2D(TP, 1, 1) == 1))
			//	cvSetZero(TP);
			//PrintMat(P);
			//PrintMat(T);

			double p11 = cvGetReal2D(TP, 0,0);		double p12 = cvGetReal2D(TP, 0,1);		double p13 = cvGetReal2D(TP, 0,2);		double p14 = cvGetReal2D(TP, 0,3);
			double p21 = cvGetReal2D(TP, 1,0);		double p22 = cvGetReal2D(TP, 1,1);		double p23 = cvGetReal2D(TP, 1,2);		double p24 = cvGetReal2D(TP, 1,3);
			double p31 = cvGetReal2D(TP, 2,0);		double p32 = cvGetReal2D(TP, 2,1);		double p33 = cvGetReal2D(TP, 2,2);		double p34 = cvGetReal2D(TP, 2,3);

			double Q11 = v*p31-p21;		double Q12 = v*p32-p22;		double Q13 = v*p33-p23;
			double Q21 = p11-u*p31;		double Q22 = p12-u*p32;		double Q23 = p13-u*p33;
			double q1 = p24-v*p34;		double q2 = u*p34-p14;
			//PrintMat(T);

			CvMat *Qi = cvCreateMat(2,3, CV_32FC1);
			CvMat *qi = cvCreateMat(2,1, CV_32FC1);
			cvSetReal2D(Qi, 0, 0, Q11);		cvSetReal2D(Qi, 0, 1, Q12);		cvSetReal2D(Qi, 0, 2, Q13);
			cvSetReal2D(Qi, 1, 0, Q21);		cvSetReal2D(Qi, 1, 1, Q22);		cvSetReal2D(Qi, 1, 2, Q23);
			cvSetReal2D(qi, 0, 0, q1);		cvSetReal2D(qi, 1, 0, q2);
			SetSubMat(&Q, 2*last, 3*iFrame, Qi);
			SetSubMat(&q, 2*last, 0, qi);
			
			
			last++;
			cvReleaseMat(&TP);
			cvReleaseMat(&P);
			cvReleaseMat(&Qi);
			cvReleaseMat(&qi);
		}
		//PrintMat(&Q);
		//PrintMat(&q);

		cvReleaseMat(&x0);
		cvReleaseMat(&x1);
		cvReleaseMat(&T);
	}


	//for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	//{
	//	for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
	//	{
	//		int frame = iCamera*nFrames + vCamera[iCamera].vTakenFrame[iFrame];
	//		vector<int>::const_iterator it = find(feature.vFrame.begin(),feature.vFrame.end(), frame);
	//		if (it != feature.vFrame.end())
	//		{
	//			int idx = (int) (it - feature.vFrame.begin());
	//			vx0.push_back(feature.vx[idx]);
	//			vy0.push_back(feature.vy[idx]);
	//			//vxInd.push_back(feature.vFrame[idx]);
	//			vxInd.push_back(frame);
	//		}
	//	}
	//}






	//for (int iCamera = 0; iCamera < vCamera.size(); iCamera++)
	//{
	//	for (int iFrame = 0; iFrame < vCamera[iCamera].vTakenFrame.size(); iFrame++)
	//	{
	//		int idx = vCamera[iCamera].vTakenFrame[iFrame];
	//		//idx = iFrame;
	//		CvMat *P = cvCreateMat(3,4,CV_32FC1);
	//		CvMat *TP = cvCreateMat(3, 4, CV_32FC1);
	//		P = cvCloneMat(vCamera[iCamera].vP[iFrame]);
	//		cvMatMul(T, P, TP);
	//		double u = cvGetReal2D(x1, iFrame, 0);
	//		double v = cvGetReal2D(x1, iFrame, 1);
	//		double p11 = cvGetReal2D(TP, 0,0);		double p12 = cvGetReal2D(TP, 0,1);		double p13 = cvGetReal2D(TP, 0,2);		double p14 = cvGetReal2D(TP, 0,3);
	//		double p21 = cvGetReal2D(TP, 1,0);		double p22 = cvGetReal2D(TP, 1,1);		double p23 = cvGetReal2D(TP, 1,2);		double p24 = cvGetReal2D(TP, 1,3);
	//		double p31 = cvGetReal2D(TP, 2,0);		double p32 = cvGetReal2D(TP, 2,1);		double p33 = cvGetReal2D(TP, 2,2);		double p34 = cvGetReal2D(TP, 2,3);

	//		double Q11 = v*p31-p21;		double Q12 = v*p32-p22;		double Q13 = v*p33-p23;
	//		double Q21 = p11-u*p31;		double Q22 = p12-u*p32;		double Q23 = p13-u*p33;
	//		double q1 = p24-v*p34;		double q2 = u*p34-p14;
	//		//PrintMat(T);
	//		//PrintMat(P);

	//		CvMat *Qi = cvCreateMat(2,3, CV_32FC1);
	//		CvMat *qi = cvCreateMat(2,1, CV_32FC1);
	//		cvSetReal2D(Qi, 0, 0, Q11);		cvSetReal2D(Qi, 0, 1, Q12);		cvSetReal2D(Qi, 0, 2, Q13);
	//		cvSetReal2D(Qi, 1, 0, Q21);		cvSetReal2D(Qi, 1, 1, Q22);		cvSetReal2D(Qi, 1, 2, Q23);
	//		cvSetReal2D(qi, 0, 0, q1);		cvSetReal2D(qi, 1, 0, q2);
	//		SetSubMat(&Q, 2*last, 3*idx, Qi);
	//		SetSubMat(&q, 2*last, 0, qi);
	//		//PrintMat(&Q);
	//		//PrintMat(&q);
	//		last++;
	//		cvReleaseMat(&TP);
	//		cvReleaseMat(&P);
	//		cvReleaseMat(&Qi);
	//		cvReleaseMat(&qi);
	//	}
	//	cvReleaseMat(&x0);
	//	cvReleaseMat(&x1);
	//	cvReleaseMat(&T);
	//}
}

void GetBB(int nFrames, int nBase, CvMat &BB)
{
	CvMat *M = cvCreateMat(nFrames, nFrames, CV_32FC1);
	GetIDCTMappingMatrix(M, nFrames);
	CvMat *B = cvCreateMat(nFrames, nBase, CV_32FC1);

	// Truncating IDCT mapping matrix
	for (int i = 0; i < nFrames; i++)
	{
		for (int j = 0; j < nBase; j++)
		{
			cvSetReal2D(B, i, j, cvGetReal2D(M, i, j));
		}
	}
	BB = *cvCreateMat(3*nFrames, 3*nBase, CV_32FC1);
	cvSetZero(&BB);
	for (int iFrame = 0; iFrame < nFrames; iFrame++)
	{
		// Create 1x(nBase) Bi^T matrix
		CvMat *BiT = cvCreateMat(1, nBase, CV_32FC1);
		cvSetZero(BiT);
		// Extract i-th row of B matrix
		GetSubMatRowwise(B, iFrame, iFrame, BiT);
		CvMat *BiTilde = cvCreateMat(3, 3*nBase, CV_32FC1);
		cvSetZero(BiTilde);
		SetSubMat(BiTilde, 0, 0, BiT);	SetSubMat(BiTilde, 1, nBase, BiT);	SetSubMat(BiTilde, 2, 2*nBase, BiT);
		SetSubMat(&BB, 3*iFrame, 0, BiTilde);
		cvReleaseMat(&BiT);
		cvReleaseMat(&BiTilde);
	}
	cvReleaseMat(&M);
	cvReleaseMat(&B);
}

void Union(vector<int> x1, vector<int> x2, vector<int> &unionx)
{
	unionx = x2;
	for (int ix1 = 0; ix1 < x1.size(); ix1++)
	{
		vector<int>::const_iterator it = find(unionx.begin(),unionx.end(), x1[ix1]);
		if (it == unionx.end())
		{
			unionx.push_back(x1[ix1]);
		}
	}
	sort(unionx.begin(), unionx.end());
}

void Intersection(vector<int> x1, vector<int> x2, vector<int> &intersectionx)
{
	for (int ix1 = 0; ix1 < x1.size(); ix1++)
	{
		vector<int>::const_iterator it = find(x2.begin(),x2.end(), x1[ix1]);
		if (it != x2.end())
		{
			intersectionx.push_back(x1[ix1]);
		}
	}
	sort(intersectionx.begin(), intersectionx.end());
}

void ScalarMul(CvMat *M, double v, CvMat *M1)
{
	for (int irow = 0; irow < M->rows; irow++)
	{
		for (int icol = 0; icol < M->cols; icol++)
			cvSetReal2D(M1, irow, icol, v*cvGetReal2D(M, irow, icol));
	}
}

void LS_homogeneous(CvMat *A, CvMat &x)
{
	x = *cvCreateMat(A->cols, 1, CV_32FC1);
	CvMat *U = cvCreateMat(A->rows, A->rows, CV_32FC1);
	CvMat *D = cvCreateMat(A->rows, A->cols, CV_32FC1);
	CvMat *V = cvCreateMat(A->cols, A->cols, CV_32FC1);
	cvSVD(A, D, U, V);
	GetSubMatColwise(V, A->cols-1, A->cols-1, &x);
	cvReleaseMat(&U);
	cvReleaseMat(&D);
	cvReleaseMat(&V);
}

void LS_homogeneous(CvMat *A, CvMat *x)
{
	//x = *cvCreateMat(A->cols, 1, CV_32FC1);
	CvMat *U = cvCreateMat(A->rows, A->rows, CV_32FC1);
	CvMat *D = cvCreateMat(A->rows, A->cols, CV_32FC1);
	CvMat *V = cvCreateMat(A->cols, A->cols, CV_32FC1);
	cvSVD(A, D, U, V);
	GetSubMatColwise(V, A->cols-1, A->cols-1, x);
	cvReleaseMat(&U);
	cvReleaseMat(&D);
	cvReleaseMat(&V);
}

void Pxx_inhomo(CvMat *P, CvMat *x, CvMat &Pxx)
{
	Pxx = *cvCreateMat(x->rows, x->cols, CV_32FC1);
	CvMat *xt = cvCreateMat(x->cols, x->rows, CV_32FC1);
	CvMat *xt_homo = cvCreateMat(x->cols+1, x->rows, CV_32FC1);
	cvTranspose(x, xt);
	CvScalar sc;	sc.val[0] = 1;
	cvSet(xt_homo, sc);
	// Make this in homogeneous
	SetSubMat(xt_homo, 0, 0, xt);
	CvMat *Pxx_homo_t = cvCreateMat(x->cols+1, x->rows, CV_32FC1);
	CvMat *Pxx_homo = cvCreateMat(x->rows, x->cols+1, CV_32FC1);
	cvMatMul(P, xt_homo, Pxx_homo_t);
	cvTranspose(Pxx_homo_t, Pxx_homo);
	GetSubMatColwise(Pxx_homo, 0, x->cols-1, &Pxx);
	cvReleaseMat(&xt);
	cvReleaseMat(&xt_homo);
	cvReleaseMat(&Pxx_homo_t);
	cvReleaseMat(&Pxx_homo);
}

void Pxx_inhomo(CvMat *P, CvMat *x, CvMat *Pxx)
{
	//Pxx = *cvCreateMat(x->rows, x->cols, CV_32FC1);
	CvMat *xt = cvCreateMat(x->cols, x->rows, CV_32FC1);
	CvMat *xt_homo = cvCreateMat(x->cols+1, x->rows, CV_32FC1);
	cvTranspose(x, xt);
	CvScalar sc;	sc.val[0] = 1;
	cvSet(xt_homo, sc);
	// Make this in homogeneous
	SetSubMat(xt_homo, 0, 0, xt);
	CvMat *Pxx_homo_t = cvCreateMat(x->cols+1, x->rows, CV_32FC1);
	CvMat *Pxx_homo = cvCreateMat(x->rows, x->cols+1, CV_32FC1);
	cvMatMul(P, xt_homo, Pxx_homo_t);
	cvTranspose(Pxx_homo_t, Pxx_homo);
	GetSubMatColwise(Pxx_homo, 0, x->cols-1, Pxx);
	cvReleaseMat(&xt);
	cvReleaseMat(&xt_homo);
	cvReleaseMat(&Pxx_homo_t);
	cvReleaseMat(&Pxx_homo);
}

void SetIndexedMatRowwise(CvMat *M, vector<int> vIndex, CvMat *M1)
{
	// M(index,:) = M1
	for (int i = 0; i < vIndex.size(); i++)
	{
		for (int j = 0; j < M1->cols; j++)
		{
			cvSetReal2D(M, vIndex[i], j, cvGetReal2D(M1, i, j));
		}
	}
}

void SetMatRowwiseFromIndexedMat(CvMat *toM, vector<int> fromIndex, CvMat *fromM)
{
	// toM = fromM(index,:)
	for (int i = 0; i < fromIndex.size(); i++)
	{
		for (int j = 0; j < fromM->cols; j++)
		{
			cvSetReal2D(toM, i, j, cvGetReal2D(fromM, fromIndex[i], j));
		}
	}
}

void PrintAlgorithm(string str)
{
	cout << "-----------------------------------------------" << endl;
	cout << str << endl;
}

double NormL2(CvMat *M)
{
	double sumM = 0.0;
	for (int i = 0; i < M->rows; i++)
	{
		for (int j = 0; j < M->cols; j++)
		{
			sumM += cvGetReal2D(M, i, j)*cvGetReal2D(M, i, j);
		}
	}
	return sqrt(sumM);
}

void Rotation2Quaternion(CvMat *R, CvMat &q)
{
	double r11 = cvGetReal2D(R,0,0);	double r12 = cvGetReal2D(R,0,1);	double r13 = cvGetReal2D(R,0,2);
	double r21 = cvGetReal2D(R,1,0);	double r22 = cvGetReal2D(R,1,1);	double r23 = cvGetReal2D(R,1,2);
	double r31 = cvGetReal2D(R,2,0);	double r32 = cvGetReal2D(R,2,1);	double r33 = cvGetReal2D(R,2,2);

	double qw = sqrt(abs(1.0+r11+r22+r33))/2;
	double qx, qy, qz;
	if (qw > QW_ZERO)
	{
		qx = (r32-r23)/4/qw;
		qy = (r13-r31)/4/qw;
		qz = (r21-r12)/4/qw;
	}
	else
	{
		double d = sqrt((r12*r12*r13*r13+r12*r12*r23*r23+r13*r13*r23*r23));
		qx = r12*r13/d;
		qy = r12*r23/d;
		qz = r13*r23/d;
	}

	q = *cvCreateMat(4,1,CV_32FC1);
	cvSetReal2D(&q, 0,0,qw);
	cvSetReal2D(&q, 1,0,qx);
	cvSetReal2D(&q, 2,0,qy);
	cvSetReal2D(&q, 3,0,qz);

	QuaternionNormalization(&q);
}

void Rotation2Quaternion(CvMat *R, CvMat *q)
{
	double r11 = cvGetReal2D(R,0,0);	double r12 = cvGetReal2D(R,0,1);	double r13 = cvGetReal2D(R,0,2);
	double r21 = cvGetReal2D(R,1,0);	double r22 = cvGetReal2D(R,1,1);	double r23 = cvGetReal2D(R,1,2);
	double r31 = cvGetReal2D(R,2,0);	double r32 = cvGetReal2D(R,2,1);	double r33 = cvGetReal2D(R,2,2);

	double qw = sqrt(abs(1.0+r11+r22+r33))/2;
	double qx, qy, qz;
	if (qw > QW_ZERO)
	{
		qx = (r32-r23)/4/qw;
		qy = (r13-r31)/4/qw;
		qz = (r21-r12)/4/qw;
	}
	else
	{
		double d = sqrt((r12*r12*r13*r13+r12*r12*r23*r23+r13*r13*r23*r23));
		qx = r12*r13/d;
		qy = r12*r23/d;
		qz = r13*r23/d;
	}

	cvSetReal2D(q, 0,0,qw);
	cvSetReal2D(q, 1,0,qx);
	cvSetReal2D(q, 2,0,qy);
	cvSetReal2D(q, 3,0,qz);

	QuaternionNormalization(q);
}


void QuaternionNormalization(CvMat *q)
{
	double qnorm = NormL2(q);
	ScalarMul(q, 1.0/qnorm, q);
}

void Quaternion2Rotation(CvMat *q, CvMat &R)
{
	QuaternionNormalization(q);
	double qw = cvGetReal2D(q, 0, 0);
	double qx = cvGetReal2D(q, 1, 0);
	double qy = cvGetReal2D(q, 2, 0);
	double qz = cvGetReal2D(q, 3, 0);
	R = *cvCreateMat(3,3,CV_32FC1);
	cvSetReal2D(&R, 0, 0, 1.0-2*qy*qy-2*qz*qz);	cvSetReal2D(&R, 0, 1, 2*qx*qy-2*qz*qw);		cvSetReal2D(&R, 0, 2, 2*qx*qz+2*qy*qw);
	cvSetReal2D(&R, 1, 0, 2*qx*qy+2*qz*qw);		cvSetReal2D(&R, 1, 1, 1.0-2*qx*qx-2*qz*qz);	cvSetReal2D(&R, 1, 2, 2*qz*qy-2*qx*qw);
	cvSetReal2D(&R, 2, 0, 2*qx*qz-2*qy*qw);		cvSetReal2D(&R, 2, 1, 2*qy*qz+2*qx*qw);		cvSetReal2D(&R, 2, 2, 1.0-2*qx*qx-2*qy*qy);
}

void Quaternion2Rotation(CvMat *q, CvMat *R)
{
	QuaternionNormalization(q);
	double qw = cvGetReal2D(q, 0, 0);
	double qx = cvGetReal2D(q, 1, 0);
	double qy = cvGetReal2D(q, 2, 0);
	double qz = cvGetReal2D(q, 3, 0);
	cvSetReal2D(R, 0, 0, 1.0-2*qy*qy-2*qz*qz);	cvSetReal2D(R, 0, 1, 2*qx*qy-2*qz*qw);		cvSetReal2D(R, 0, 2, 2*qx*qz+2*qy*qw);
	cvSetReal2D(R, 1, 0, 2*qx*qy+2*qz*qw);		cvSetReal2D(R, 1, 1, 1.0-2*qx*qx-2*qz*qz);	cvSetReal2D(R, 1, 2, 2*qz*qy-2*qx*qw);
	cvSetReal2D(R, 2, 0, 2*qx*qz-2*qy*qw);		cvSetReal2D(R, 2, 1, 2*qy*qz+2*qx*qw);		cvSetReal2D(R, 2, 2, 1.0-2*qx*qx-2*qy*qy);
}

void Vec2Skew(CvMat *m, CvMat *M)
{
	cvSetZero(M);
	cvSetReal2D(M, 0,1,-cvGetReal2D(m, 2,0));
	cvSetReal2D(M, 0,2,cvGetReal2D(m, 1,0));
	cvSetReal2D(M, 1,0,cvGetReal2D(m, 2,0));
	cvSetReal2D(M, 1,2,-cvGetReal2D(m, 0,0));
	cvSetReal2D(M, 2,0,-cvGetReal2D(m, 1,0));
	cvSetReal2D(M, 2,1,cvGetReal2D(m, 0,0));
}

double DistancePixel(double x1, double y1, double x2, double y2)
{
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void MatrixOrthogonalization(CvMat *A, CvMat *A1)
{
	CvMat *a = cvCreateMat(A->rows, 1, CV_32FC1);
	cvSetZero(A1);
	GetSubMatColwise(A, 0, 0, a);
	ScalarMul(a, 1/NormL2(a), a);
	SetSubMat(A1, 0, 0, a);
	
	for (int icol = 1; icol < A->cols; icol++)
	{
		CvMat *v0 = cvCreateMat(A->rows, 1, CV_32FC1);
		CvMat *v1 = cvCreateMat(A->rows, 1, CV_32FC1);
		CvMat *A_ = cvCreateMat(A->rows, icol, CV_32FC1);
		GetSubMatColwise(A1, 0, icol-1, A_);

		GetSubMatColwise(A, icol, icol, v0);
		ScalarMul(v0, 1/NormL2(v0), v0);

		GramSchmidt(A_, v0, v1);

		SetSubMat(A1, 0, icol, v1);
		cvReleaseMat(&v1);
		cvReleaseMat(&v0);
		cvReleaseMat(&A_);
	}
	cvReleaseMat(&a);
}

void GramSchmidt(CvMat *A, CvMat *v0, CvMat *v1)
{
	GetSubMatColwise(v0, 0, 0, v1);
	for (int icol = 0; icol < A->cols; icol++)
	{
		CvMat *a = cvCreateMat(A->rows, 1, CV_32FC1);
		for (int irow = 0; irow < a->rows; irow++)
			cvSetReal2D(a, irow, 0, cvGetReal2D(A, irow, icol));
		CvMat *Projv = cvCreateMat(v1->rows, 1, CV_32FC1);
		ProjOntoVector(v0, a, Projv);
		cvSub(v1, Projv, v1);

		cvReleaseMat(&a);
		cvReleaseMat(&Projv);
	}
}

void ProjOntoVector(CvMat *v, CvMat *u, CvMat *Projv)
{
	CvMat *vt = cvCreateMat(1, v->rows, CV_32FC1);
	CvMat *ut = cvCreateMat(1, u->rows, CV_32FC1);
	cvTranspose(v, vt);
	cvTranspose(u, ut);
	CvMat *dot1 = cvCreateMat(1,1,CV_32FC1);
	CvMat *dot2 = cvCreateMat(1,1,CV_32FC1);
	cvMatMul(vt, u, dot1);
	cvMatMul(ut, u, dot2);
	double dot1d = cvGetReal2D(dot1, 0, 0);
	double dot2d = cvGetReal2D(dot2, 0, 0);
	ScalarMul(u, dot1d/dot2d, Projv);

	cvReleaseMat(&vt);
	cvReleaseMat(&ut);
	cvReleaseMat(&dot1);
	cvReleaseMat(&dot2);
}

void GetCameraParameter(CvMat *P, CvMat *K, CvMat &R, CvMat &C)
{
	R = *cvCreateMat(3,3,CV_32FC1);
	C = *cvCreateMat(3,1,CV_32FC1);
	CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
	CvMat *invK = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K, invK);
	cvMatMul(invK, P, temp34);
	GetSubMatColwise(temp34, 0,2,&R);
	GetSubMatColwise(temp34, 3,3,&C);
	CvMat *invR = cvCreateMat(3,3,CV_32FC1);
	cvInvert(&R, invR);
	cvMatMul(invR, &C, &C);
	ScalarMul(&C, -1, &C);

	cvReleaseMat(&temp34);
	cvReleaseMat(&invK);
	cvReleaseMat(&invR);
}

void GetCameraParameter(CvMat *P, CvMat *K, CvMat *R, CvMat *C)
{
	//R = *cvCreateMat(3,3,CV_32FC1);
	//C = *cvCreateMat(3,1,CV_32FC1);
	CvMat *temp34 = cvCreateMat(3,4,CV_32FC1);
	CvMat *invK = cvCreateMat(3,3,CV_32FC1);
	cvInvert(K, invK);
	cvMatMul(invK, P, temp34);
	GetSubMatColwise(temp34, 0,2,R);
	GetSubMatColwise(temp34, 3,3,C);
	CvMat *invR = cvCreateMat(3,3,CV_32FC1);
	cvInvert(R, invR);
	cvMatMul(invR, C, C);
	ScalarMul(C, -1, C);

	cvReleaseMat(&temp34);
	cvReleaseMat(&invK);
	cvReleaseMat(&invR);
}

void GetCameraMatrix(CvMat *K, CvMat *R, CvMat *C, CvMat *P)
{
	CvMat *trans = cvCreateMat(3,4,CV_32FC1);
	CvMat *C_ = cvCreateMat(3,1,CV_32FC1);
	cvSetIdentity(trans);
	ScalarMul(C, -1, C_);
	SetSubMat(trans, 0, 3, C_);
	cvMatMul(R, trans, trans);
	cvMatMul(K, trans, P);
	cvReleaseMat(&trans);
	cvReleaseMat(&C_);
}

double IDCTContinuous(int max_nFrames, double t, vector<double> theta)
{
	CvMat *Bt = cvCreateMat(1, theta.size(), CV_32FC1);
	cvSetZero(Bt);
	cvSetReal2D(Bt, 0, 0, sqrt(1.0/max_nFrames));
	for (int i = 1; i < theta.size(); i++)
	{
		cvSetReal2D(Bt, 0, i, sqrt(2.0/max_nFrames)*cos((double)(2*t+1)*(i)*PI/2/max_nFrames));
	}
	CvMat *theta_vec = cvCreateMat(theta.size(), 1, CV_32FC1);
	for (int i = 0; i < theta.size(); i++)
	{
		cvSetReal2D(theta_vec, i, 0, theta[i]);
	}
	CvMat *time = cvCreateMat(1,1, CV_32FC1);
	cvMatMul(Bt, theta_vec, time);
	double traj = cvGetReal2D(time, 0, 0);
	cvReleaseMat(&Bt);
	cvReleaseMat(&theta_vec);
	cvReleaseMat(&time);
	return traj;
}

void GetIDCTContinuousMatrix(int max_nFrames, int frequency, int nBase, CvMat &BBt)
{
	int k = max_nFrames;
	BBt = *cvCreateMat(max_nFrames*frequency, nBase, CV_32FC1);
	for (int irow = 0; irow < BBt.rows; irow++)
	{
		cvSetReal2D(&BBt, irow, 0, sqrt(1.0/k));
	}
	for (int irow = 0; irow < BBt.rows; irow++)
	{
		double t = (double)1.0/(double)frequency*irow;
		for (int icol = 1; icol < BBt.cols; icol++)
		{			
			cvSetReal2D(&BBt, irow, icol, sqrt(2.0/k)*cos((double)(2*t+1)*icol*PI/2/k));
		}
	}
}