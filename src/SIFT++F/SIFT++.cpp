// SIFT++.cpp : Defines the entry point for the console application.
//

#include "sift.hpp"

#include<cv.h>
#include<cxcore.h>
#include<highgui.h>
#include<string>
#include<iostream>
#include<iomanip>
#include<fstream>
#include<sstream>
#include<algorithm>
#include<stdint.h>
#include<memory>
#include<math.h>

using namespace std ;


std::ostream&
insertDescriptor(std::ostream& os,
                 VL::float_t const * descr_pt,
                 bool binary,
                 bool fp )
{
#define RAW_CONST_PT(x) reinterpret_cast<char const*>(x)
#define RAW_PT(x)       reinterpret_cast<char*>(x)

	if( fp ) 
	{

		/* convert to 32 bits floats (single precision) */
		VL::float32_t fdescr_pt [128] ;
		for(int i = 0 ; i < 128 ; ++i)
			fdescr_pt[i] = VL::float32_t( descr_pt[i]) ;

		if( binary ) 
		{
			/* 
				Test for endianess. Recall: big_endian = the most significant
				byte at lower memory address.
			*/
			short int const word = 0x0001 ;
			bool little_endian = RAW_CONST_PT(&word)[0] ;
      
			/* 
				We save in big-endian (network) order. So if this machine is
				little endiand do the appropriate conversion.
			*/
			if( little_endian ) 
			{
				for(int i = 0 ; i < 128 ; ++i) {
					VL::float32_t tmp = fdescr_pt[ i ] ;        
					char* pt  = RAW_PT(fdescr_pt + i) ;
					char* spt = RAW_PT(&tmp) ;
					pt[0] = spt[3] ;
					pt[1] = spt[2] ;
					pt[2] = spt[1] ;
					pt[3] = spt[0] ;
				}
			}            
			os.write( RAW_PT(fdescr_pt), 128 * sizeof(VL::float32_t) ) ;

		} 
		else 
		{

			for(int i = 0 ; i < 128 ; ++i) 
			os << ' ' 
				<< fdescr_pt[i] ;
		}

	}
	else 
	{
		VL::uint8_t idescr_pt [128] ;

		for(int i = 0 ; i < 128 ; ++i)
			idescr_pt[i] = uint8_t(VL::float_t(512) * descr_pt[i]) ;
    
		if( binary ) 
		{
			os.write( RAW_PT(idescr_pt), 128) ;	

		}
		else
		{       
			for(int i = 0 ; i < 128 ; ++i) 
			os << ' ' 
				<< uint32_t( idescr_pt[i] ) ;
		}
	}
	return os ;
}

/* keypoint list */
typedef vector<pair<VL::Sift::Keypoint,VL::float_t> > Keypoints ;

// -------------------------------------------------------------------
//                                                                main
// -------------------------------------------------------------------
int main(int argc, char** argv)
{
	int    first          = -1 ;
	int    octaves        = -1 ;
	int    levels         = 3 ;
	float  threshold      = 0.04f / levels / 2.0f ;
	float  edgeThreshold  = 10.0f;
	float  magnif         = 3.0 ;
	int    nodescr        = 0 ;
	int    noorient       = 0 ;
	int    stableorder    = 0 ;
	int    savegss        = 0 ;
	int    verbose        = 0 ;
	int    binary         = 0 ;
	int    haveKeypoints  = 0 ;
	int    unnormalized   = 0 ;
	int    fp             = 0 ;
	string outputFilenamePrefix ;
	string outputFilename ;
	string descriptorsFilename ;
	string keypointsFilename ;

	//string name = "E:/SIFT_test/1/image0000017.bmp";
	//string key_file = "E:/SIFT_test/1/image0000017.key";

    string name = argv[1] ;
	VL::PgmBuffer buffer ;
	string key_file = argv[2];
	//ifstream in(name.c_str(), ios::binary); 
	//extractPgm(in, buffer) ;
	IplImage * gray = cvLoadImage(name.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

	buffer.height = gray->height;
	buffer.width = gray->width;

	VL::pixel_t *im_pt = new VL::pixel_t [ buffer.width*buffer.height ];
	for (int j = 0; j < gray->height; j++)
	{
		for (int i = 0; i < gray->width; i++)
		{
			im_pt[j*gray->width+i] = ((uchar *)(gray->imageData + j*gray->widthStep))[i]/255.0f;
		}
	}
	buffer.data = im_pt;
	cvReleaseImage(&gray);



	int         O      = octaves ;    
    int const   S      = levels ;
    int const   omin   = first ;
    float const sigman = .5 ;
    float const sigma0 = 1.6 * powf(2.0f, 1.0f / S) ;

	O = std::max(int(std::floor(log((float)std::min(buffer.width,buffer.height))/log(2.0)) - omin -3), 1) ;
	VL::Sift sift(buffer.data, buffer.width, buffer.height, 
		    sigman, sigma0,
		    O, S,
		    omin, -1, S+1) ;
	sift.detectKeypoints(threshold, edgeThreshold) ;
	sift.setNormalizeDescriptor( ! unnormalized ) ;
    sift.setMagnification( magnif ) ;
		         
	// open output file
	ofstream out(key_file.c_str(), ios::binary) ;       
	out.flags(ios::fixed) ;

	// -------------------------------------------------------------
	//            Run detector, compute orientations and descriptors
	// -------------------------------------------------------------

	int count = 0;
	
	for( VL::Sift::KeypointsConstIter iter = sift.keypointsBegin() ; iter != sift.keypointsEnd() ; ++iter ) 
	{    
		// detect orientations
		VL::float_t angles [4] ;
		int nangles ;
		if( ! noorient ) {
			nangles = sift.computeKeypointOrientations(angles, *iter) ;
		} 
		else 
		{
			nangles = 1;
			angles[0] = VL::float_t(0) ;
		}
	    
		auto_ptr<ofstream> descriptorsOut_pt ;
		// compute descriptors
		for(int a = 0 ; a < nangles ; ++a) 
		{
			count++;
		}
	}
	out << count << " 128" << endl;
	for( VL::Sift::KeypointsConstIter iter = sift.keypointsBegin() ; iter != sift.keypointsEnd() ; ++iter ) 
	{    
		// detect orientations
		VL::float_t angles [4] ;
		int nangles ;
		if( ! noorient ) {
			nangles = sift.computeKeypointOrientations(angles, *iter) ;
		} 
		else 
		{
			nangles = 1;
			angles[0] = VL::float_t(0) ;
		}
	    
		auto_ptr<ofstream> descriptorsOut_pt ;
		// compute descriptors
		for(int a = 0 ; a < nangles ; ++a) 
		{
			out << setprecision(2) << iter->y << ' '
			<< setprecision(2) << iter->x << ' '
			<< setprecision(2) << iter->sigma << ' ' 
			<< setprecision(3) << angles[a] ;

			/* compute descriptor */
			VL::float_t descr_pt [128] ;
			sift.computeKeypointDescriptor(descr_pt, *iter, angles[a]) ;
	
			/* save descriptor to to appropriate file */	      
			if( ! nodescr ) 
			{
				if( descriptorsOut_pt.get() ) 
				{
					ostream& os = *descriptorsOut_pt.get() ;
					insertDescriptor(os, descr_pt, true, fp) ;
				} 
				else 
				{
					insertDescriptor(out, descr_pt, false, fp) ;
				}
			}/* next line */
			out << endl ;
		} // next angle
	}// next keypoint
	out.close();

	cout << count<< " points are saved to " << argv[2] << endl;
  
  return 0 ;
}
