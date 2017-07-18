Structure from motion code
Written by Hyun Soo Park

This code is for a GoPro 3 camera (fisheye distortion)

INPUT: + image/image%07d.bmp: images (bmp format) in image folder
	   + calib_fisheye.txt: intrinsic calibration file
OUTPUT: + camera.txt: extrinsic camera pose file
        + structure.txt: 3D points with rgb color
        
Before execution, change path of the calibration file (first line in Reconstruction.sh file)

Execution:
Given INPUTs, run a script, Reconstruction.sh
>> cd image
>> ffmpeg -i gopro.mp4 image/image%07d.bmp (image extraction from a video using ffmpeg)
>> cp (calib_fisheye.txt path) image/
>> Reconstruction.sh

Requirements:
+ OpenCV
+ Boost
+ Ceres (http://ceres-solver.org/)

Compilation:
+ Go to each code folder and run cmake.
>> cmake .
>> make
+ You may want to modify the CMakerLists.txt file for path.
+ Place the executable to a path defined in environment variables.

Code detail:
+ Feature extraction: SIFT++ 
	This code extracts key points given an image.
	The original code available from http://www.robots.ox.ac.uk/~vedaldi/code/siftpp.html
+ Feature matching: Matching_sift
	This code matches features between all possible pairs of images.
+ Building tracks: Stitching_sift
	This code associates features to produce a long term track.
+ Initial frame detection: InitialFrameDetection
	This code computes the best initial pair to start structure from motion
+ Structure from motion: Reconstruction_ceres_gopro3
	This code takes matchings to produce 3D point cloud and camera poses using bundle adjustment
	
