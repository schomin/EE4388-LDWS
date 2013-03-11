/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 7 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#if !defined LINEF
#define LINEF

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define PI 3.1415926

class LineFinder {

  private:

	  // original image
	  cv::Mat img;


	cv::Point p11;
	cv::Point p21;
	cv::Point p31;
	cv::Point p41;
	  // vector containing the end points 
	  // of the detected lines
	  std::vector<cv::Vec4i> lines;

	  // accumulator resolution parameters
	  double deltaRho;
	  double deltaTheta;

	  // minimum number of votes that a line 
	  // must receive before being considered
	  int minVote;

	  // min length for a line
	  double minLength;

	  // max allowed gap along the line
	  double maxGap;

  public:

	  // Default accumulator resolution is 1 pixel by 1 degree
	  // no gap, no mimimum length
	  LineFinder() : deltaRho(1), deltaTheta(PI/180), minVote(10), minLength(0.), maxGap(0.) {}

	  // Set the resolution of the accumulator
	  void setAccResolution(double dRho, double dTheta) {

		  deltaRho= dRho;
		  deltaTheta= dTheta;
	  }

	  // Set the minimum number of votes
	  void setMinVote(int minv) {

		  minVote= minv;
	  }

	  // Set line length and gap
	  void setLineLengthAndGap(double length, double gap) {

		  minLength= length;
		  maxGap= gap;
	  }

	  // Apply probabilistic Hough Transform
	  std::vector<cv::Vec4i> findLines(cv::Mat& binary) {

		  lines.clear();
		  cv::HoughLinesP(binary,lines,deltaRho,deltaTheta,minVote, minLength, maxGap);

		  return lines;
	  }
	  
	  // Set a different set of lines
	  void setLines(std::vector<cv::Vec4i> li) {
	  
	  		lines = li;
	  
	  }
	  
	  // Set a different set of lines
	  std::vector<cv::Vec4i> getLines() {
	  
	  		return lines;
	  
	  }
	  
	  void shiftLines(float delta){
	  
	  	  // Shift the lines
		  std::vector<cv::Vec4i>::iterator it2= lines.begin();
	
		  while (it2!=lines.end()) {
		  
		  		std::cout << "before shift: " << (*it2)[0] << "\n";
		
			  (*it2)[0] = (*it2)[0] + floor0(delta);
			  (*it2)[2] = (*it2)[2] + floor0(delta);
			  
			  std::cout << "after shift: " << (*it2)[0] << "\n";
		
			  ++it2;	
		  }
	  
	  }
	  
	  	double floor0( float value )
		{
			if (value < 0.0)
				return ceil( value );
			else
				return floor( value );
		}

	  // Draw the detected lines on an image
	  void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(0,0,200)) {
	
		  // Draw the lines
		  std::vector<cv::Vec4i>::const_iterator it2= lines.begin();
	
			int height = image.size().height;
			int width = image.size().width;
	

			double leftmostx = -1;
			double leftmosty = -1;
			double rightmostx = -1;
			double rightmosty = -1;
		  while (it2!=lines.end()) {
		  
			  cv::Point pt1((*it2)[0]+(width*0.167),(*it2)[1] + height/2);
			  cv::Point pt2((*it2)[2]+(width*0.167),(*it2)[3] + height/2);

			  
			  double midx = (double)((*it2)[0] + (*it2)[2])/2;
			  double midy = (double)((*it2)[1] + (*it2)[3])/2;
			  
				  if ( (leftmosty < midy || leftmosty == -1)  && midx < width/2 && midx > 30) {
				  
						p11 = pt1;
						p21 = pt2;
						leftmostx = midx;
						leftmosty = midy;
				  
				  }
				  if ((rightmosty < midy || rightmosty == -1) && midx > width/2) {
				  
						p31 = pt1;
						p41 = pt2;
						rightmostx = midx;
						rightmosty = midy;
				  
				  }
			  cv::line( image, pt1, pt2, cv::Scalar(0,200,0), 4);
			  ++it2;	
		  }
		  
		   cv::Point pt1(width/2,0);        
			  cv::Point pt2(width/2,height);
			  
			  cv::line( image, pt1, pt2, color, 4);
		  
		  
		  int lineType = 8;

		   /** Create some points */
		   cv::Point rook_points[1][4];
		   rook_points[0][0] = p11;
		   rook_points[0][1] = p21;
		   rook_points[0][2] = p31;
		   rook_points[0][3] = p41;
		
		   const cv::Point* ppt[1] = { rook_points[0] };
		   int npt[] = { 4 };
		
		   cv::fillPoly( image,
					 ppt,
					 npt,
					 1,
					 color,
					 lineType );
	  }

	  // Eliminates lines that do not have an orientation equals to
	  // the ones specified in the input matrix of orientations
	  // At least the given percentage of pixels on the line must 
	  // be within plus or minus delta of the corresponding orientation
	  std::vector<cv::Vec4i> removeLinesOfInconsistentOrientations(
		  const cv::Mat &orientations, double percentage, double delta, cv::Mat &image) {

			  std::vector<cv::Vec4i>::iterator it= lines.begin();
				int height = image.size().height;
			int width = image.size().width;
			  // check all lines
			  while (it!=lines.end()) {

				  // end points
				  int x1= (*it)[0];
				  int y1= (*it)[1];
				  int x2= (*it)[2];
				  int y2= (*it)[3];
		   
				  // line orientation + 90o to get the parallel line
				  double ori1= atan2(static_cast<double>(y1-y2),static_cast<double>(x1-x2))+PI/2;
				  if (ori1>PI) ori1= ori1-2*PI;

				  double ori2= atan2(static_cast<double>(y2-y1),static_cast<double>(x2-x1))+PI/2;
				  if (ori2>PI) ori2= ori2-2*PI;
	
				  // for all points on the line
				  cv::LineIterator lit(orientations,cv::Point(x1,y1),cv::Point(x2,y2));
				  int i,count=0;
				  for(i = 0, count=0; i < lit.count; i++, ++lit) { 
		
					  float ori= *(reinterpret_cast<float *>(*lit));

					  // is line orientation similar to gradient orientation ?
					  if (std::min(fabs(ori-ori1),fabs(ori-ori2))<delta)
						  count++;
		
				  }

				  double consistency= count/static_cast<double>(i);
				  
				  
				  // Added by Andrew
				  
				  // Remove lines that are found to be horizontal
				  
				  double angle = atan2(static_cast<double>(y2-y1),static_cast<double>(x2-x1)) *180/PI;
				  
				  std::cout << "Angle: " << angle << "\n";
				  
				  if (angle < 8 && angle > -8){
				  	(*it)[0]=(*it)[1]=(*it)[2]=(*it)[3]=0;
				  }
				  
				  //Remove lines that are outside of the realm of the current lane
				  
				  double midx = (double)((*it)[0] + (*it)[2])/2;
			  	  double midy = (double)((*it)[1] + (*it)[3])/2;
			  	  
			  	  if (midy < width * 0.12){
			  	  	(*it)[0]=(*it)[1]=(*it)[2]=(*it)[3]=0;
			  	  }
				  
				  ///end
				  

				  // set to zero lines of inconsistent orientation
				  if (consistency < percentage) {
					  (*it)[0]=(*it)[1]=(*it)[2]=(*it)[3]=0;
				  }

				  ++it;
			  }

			  return lines;
	  }
};


#endif
