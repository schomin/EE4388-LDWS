#include <stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "linefinder.h"
#include "edgedetector.h"

using namespace cv;
int col1 = 0;
int col2 = 0;
int x1 = 0;
int y1 = 0;

/** @function main */
int main( int argc, char** argv )
{
	//-- CV Capture object for camera
	CvCapture* capture;
	//-- Frame Captured from capture object
   	Mat frame;
 
 	//-- Start capture from default camera
	capture = cvCaptureFromCAM( -1 );
	//-- If capture was successful
 	if( capture )
  	{ 
		cv::namedWindow("Detected Lines (2)");
		//-- While this program is running
		while( true )
		{
			//-- Get a frame from the capture
			frame = cvQueryFrame( capture );
			//-- If fram is not empty
			if( !frame.empty() ){

					cv::Mat image = frame;
					if (!image.data)
						return 0; 

					// Compute Sobel
					EdgeDetector ed;
					ed.computeSobel(image);


					// Apply Canny algorithm
					cv::Mat contours;
					cv::Canny(image,contours,125,350);
					cv::Mat contoursInv;
					cv::threshold(contours,contoursInv,128,255,cv::THRESH_BINARY_INV);


					// Hough tranform for line detection
					std::vector<cv::Vec2f> lines;
					cv::HoughLines(contours,lines,1,PI/180,60);

					// Draw the lines
					cv::Mat result(contours.rows,contours.cols,CV_8U,cv::Scalar(255));
					image.copyTo(result);

					std::cout << "Lines detected: " << lines.size() << std::endl;

					std::vector<cv::Vec2f>::const_iterator it= lines.begin();

					while (it!=lines.end()) {

						float rho= (*it)[0];   // first element is distance rho
						float theta= (*it)[1]; // second element is angle theta

						if (theta < PI/4. || theta > 3.*PI/4.) { // ~vertical line

							// point of intersection of the line with first row
							cv::Point pt1(rho/cos(theta),0);        
							// point of intersection of the line with last row
							cv::Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
							// draw a white line
							cv::line( result, pt1, pt2, cv::Scalar(200,0,0), 10); 

						} else { // ~horizontal line

							// point of intersection of the line with first column
							cv::Point pt1(0,rho/sin(theta));        
							// point of intersection of the line with last column
							cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
							// draw a white line
							cv::line( result, pt1, pt2, cv::Scalar(200,0,0), 10); 
						}

						std::cout << "line: (" << rho << "," << theta << ")\n"; 

						++it;
					}


					// Display the detected line image
					//cv::namedWindow("Detected Lines with Hough");
					//cv::imshow("Detected Lines (2)",result);

					// Create LineFinder instance
					LineFinder ld;

					// Set probabilistic Hough parameters
					ld.setLineLengthAndGap(100,20);
					ld.setMinVote(80);

					// Detect lines
					std::vector<cv::Vec4i> li= ld.findLines(contours);
					ld.drawDetectedLines(image);

					// eliminate inconsistent lines
					ld.removeLinesOfInconsistentOrientations(ed.getOrientation(),0.4,0.1);

				   // Display the detected line image
					image= frame;
					//ld.drawDetectedLines(image);
					


					if(col1 == 1)
						{					
							cv::circle(image,cv::Point(230,444), 20, cv::Scalar(0,0,255,255),-1,8,0);
						}
					else
						{					
							cv::circle(image,cv::Point(230,444), 20, cv::Scalar(255,0,0,255),-1,8,0);
						}		

					
					if(col2 == 1)
						{					
							cv::circle(image,cv::Point(380,444), 20, cv::Scalar(0,0,255,255),-1,8,0);
						}
					else
						{					
							cv::circle(image,cv::Point(380,444), 20, cv::Scalar(255,0,0,255),-1,8,0);
						}



					//cv::namedWindow("Detected Lines (2)");
					cv::imshow("Detected Lines (2)",image);

		/*			// Create a Hough accumulator
					cv::Mat acc(200,180,CV_8U,cv::Scalar(0));

					// Choose a point
					int x=50, y=30;

					// loop over all angles
					for (int i=0; i<180; i++) 
					{

						double theta= i*PI/180.;

						// find corresponding rho value 
						double rho= x*cos(theta)+y*sin(theta);
						int j= static_cast<int>(rho+100.5);

						std::cout << i << "," << j << std::endl;

						// increment accumulator
						acc.at<uchar>(j,i)++;
					}
*/
			}
			else{
				printf(" --(!) No captured frame -- Break!"); break; 
			}
			int c = waitKey(10);
			if( (char)c == 'c' ) { break; };
		}
	}
  return 0;
  }

