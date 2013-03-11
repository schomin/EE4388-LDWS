

#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "linefinder.h"
#include "edgedetector.h"

using namespace cv;



/*READ ME FIRST:

The stuff I changed is assuming that only subsequent lines will be parallel, I don't know why I think this will be the case
but I kind of saw it as a safe guard in protecting against a line from the edge of the road being parallel to a lane divider.
If we want the current slope to scan through all previously saved slopes we will need to embed another loop inside the if 
statement. Unless I was just cleaning up the code I typically put a comment with my name at the end of it where I changed things.
Also, the code only stores the points if the slopes are exactly equal. We will probably need to put in some wiggle room later but
for the first semester stuff I didn't bother.

*/




/** @function main */
int main( int argc, char** argv ){
	//-- CV Capture object for camera
	CvCapture* capture;
	//-- Frame Captured from capture object
	Mat frame;
	Mat lastFrame;
	//-- Start capture from default camera
	capture =  cvCaptureFromFile( "./video1.mp4" );
	//capture =  cvCaptureFromFile( "./IMG_1097.MOV" );
	//capture =  cvCaptureFromFile( "./Video Feb 01, 4 31 02 PM.mov" );
	//-- If capture was successful
  if( capture )
   {
	cv::namedWindow("Detected Lanes");
	cvSetWindowProperty("Detected Lanes", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	int linesDetected = 20;
	//-- While this program is running
	//futline is used to store the slope of past lines in order to compare them between each other
	float futline[linesDetected];
	//the x coordinates of stored lines 
	float x[2*(linesDetected)];  
	//the y coordinates of stored lines
	float y[2*(linesDetected)];  
	std::vector<cv::Vec4i> lastLines;
	int b = 0; 
	int frameCount = 0;
	// Create LineFinder instance
	LineFinder ld;
	
	// Set probabilistic Hough parameters  ---- we can fiddle with this later in order to improve accuracy (2nd semester) ...Alex
	ld.setLineLengthAndGap(40,5);
	ld.setMinVote(10);
	while( true )
	{
		//-- Get a frame from the capture
		frame = cvQueryFrame( capture );
		//-- If fram is not empty
		if( !frame.empty() ){

		cv::Mat image;
		cv::resize(frame, image, Size(), 0.5, 0.5, INTER_NEAREST);
		cv::Mat currentFrame;
		cv::resize(frame, currentFrame, Size(), 0.5, 0.5, INTER_NEAREST);
		int height    = image.size().height;
  		int width     = image.size().width;		
  		cv::Rect myROI(width*0.167, height/2, width*.833, height/2);
		image = image(myROI);
		if (!image.data)
		return 0;

		// Compute Sobel
		EdgeDetector ed;
		ed.computeSobel(image);


		// Apply Canny algorithm
		cv::Mat contours;
		//cv::Canny(image,contours,125,350);
		cv::Canny(image, contours, 50, 200);
		cv::Mat contoursInv;
		cv::threshold(contours,contoursInv,128,255,cv::THRESH_BINARY_INV);

		cv::imshow("Edges",contours);

		// Hough tranform for line detection
		std::vector<cv::Vec2f> lines;
		//cv::HoughLines(contours,lines,1,PI/180,60);
		
		cv::HoughLines(contours,lines,1,PI/180,10);

		// Draw the lines
		cv::Mat result(contours.rows,contours.cols,CV_8U,cv::Scalar(255));
		image.copyTo(result);

		//std::cout << "Lines detected: " << lines.size() << "\n";

		//Define stuff here
		//slope is used to determine if two lines are parallel
		float slope[lines.size()];  
		//incrementer for the slope, x/y, and futline arrays
		int f = 0; 
		int e = 0; 
		//used for the line and circle intersections
		int col1 = 0;
		int col2 = 0;
		
		std::vector<cv::Vec2f>::const_iterator it= lines.begin();

		while (it!=lines.end()) {   

				float rho= (*it)[0]; // first element is distance rho
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

				//std::cout << "line: (" << rho << "," << theta << ")\n";

				++it;
		}



		// Detect lines
		std::vector<cv::Vec4i> li = ld.findLines(contours);
		
		// eliminate inconsistent lines   ----maybe use the slopes found within the while loops to eliminate certain lines?? ...Alex
		li = ld.removeLinesOfInconsistentOrientations(ed.getOrientation(),0.4,0.15, image);

		std::vector<cv::Vec4i>::const_iterator it2= li.begin();
		
		int li_size = 0;
		
		while (it2!=li.end()) { 
		
			if ((*it2)[0] > 30 && (*it2)[1] > 30 && (*it2)[2] >30  && (*it2)[3] > 30){			
				
				std::cout << "0 : " << (*it2)[0] << "\n";
				std::cout << "1 : " << (*it2)[1] << "\n";
				std::cout << "2 : " << (*it2)[2] << "\n";
				std::cout << "3 : " << (*it2)[3] << "\n";
				li_size++;
			
			}
		
			++it2;
		
		}
		

		it2= li.begin();

		//if there are no lines use the coordinates stored in the x and y arrays
		/*if((it2==li.end() || li_size < 2) && !lastLines.empty()){
				// Detect the features
				int minHessian = 300;
				
				SurfFeatureDetector detector( minHessian );
				
				std::vector<KeyPoint> keypoints_1, keypoints_2;
				
				detector.detect( lastFrame, keypoints_1 );
				detector.detect( currentFrame, keypoints_2 );
				
				 //-- Step 2: Calculate descriptors (feature vectors)
				SurfDescriptorExtractor extractor;
				
				Mat descriptors_1, descriptors_2;
				
				extractor.compute( lastFrame, keypoints_1, descriptors_1 );
				extractor.compute( currentFrame, keypoints_2, descriptors_2 );
				
				//-- Step 3: Matching descriptor vectors using FLANN matcher
				std::vector< DMatch > matches;
				float deltax = 0;
				try
				{
					FlannBasedMatcher matcher;
					matcher.match( descriptors_1, descriptors_2, matches );
				
					double max_dist = 0; 
					double min_dist = 100;
					
					//-- Quick calculation of max and min distances between keypoints
					for( int i = 0; i < descriptors_1.rows; i++ )
					{ 
						double dist = matches[i].distance;
						if( dist < min_dist ) 
							min_dist = dist;
						if( dist > max_dist ) 
							max_dist = dist;
					}
					
					printf("-- Max dist : %f \n", max_dist );
					printf("-- Min dist : %f \n", min_dist );
					
					//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
					//-- PS.- radiusMatch can also be used here.
					std::vector< DMatch > good_matches;
					
					int descriptorcount = 0;
					
					for( int i = 0; i < descriptors_1.rows; i++ )
					{ 
						if( matches[i].distance < 2*min_dist && descriptorcount <= 8)
						{ 
							good_matches.push_back( matches[i]); 
							descriptorcount++;
						}
					}
					
					//-- Draw only "good" matches
					Mat img_matches;
					drawMatches( lastFrame, keypoints_1, currentFrame, keypoints_2,
							   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
							   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
					
					//-- Show detected matches
					//imshow( "Good Matches", img_matches );
					
					float deltaxsum = 0;
					float matchcount = 0;
					
					for( int i = 0; i < good_matches.size(); i++ )
					{ 
					
						printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
					
						std::cout << "point: " << keypoints_1[good_matches[i].queryIdx].pt.x << ", " << keypoints_2[good_matches[i].trainIdx].pt.x << "\n";
						std::cout << "delta: " << keypoints_1[good_matches[i].queryIdx].pt.x - keypoints_2[good_matches[i].trainIdx].pt.x << "\n";
						
						deltaxsum += keypoints_2[good_matches[i].trainIdx].pt.x - keypoints_1[good_matches[i].queryIdx].pt.x;
						matchcount++;
		
					}
					
					deltax = deltaxsum/matchcount;
					
					std::cout << "avgdelta: " << deltax << "\n";
					
				}
				catch( cv::Exception& e )
				{
					printf("Error Caught\n");
				}
								
				//changed this to a for loop instead of a while loop so we could use the multiple parallel lines stored in the futline and x/y arrays...Alex
				for (int g = 0; g < b; g +=2 ) { //yet another new integer for counting purposes...Alex
						//uses the arrays with stored past values instead of the line vector
						float x1 = x[g] + ld.floor0(deltax);
						float x2 = x[g+1] + ld.floor0(deltax); 
						float y1 = y[g];
						float y2 = y[g+1];
												
						x[g] = x1;
						x[g+1] = x2;
						y[g] = y1;
						y[g+1] = y2;

						float abdist = sqrt(pow((x2-x1),2)+pow((y2-y1),2));

						// compute the direction vector D from A to B
						float dx = (x2-x1)/abdist;
						float dy = (y2-y1)/abdist;

						//I dont think we need this here...Alex
						//slope[f] = dx/dy;

						// compute the value t of the closest point to the circle 1 and circle 2 center
						float t1 = dx*(230-x1) + dy*(444-y1);
						float t2 = dx*(380-x1) + dy*(444-y1);

						// This is the projection of C on the line from A to B.

						// compute the coordinates of the point E on line and closest to C1 and C2
						float ex1 = t1*dx+x1;
						float ey1 = t1*dy+y1;
						float ex2 = t2*dx+x1;
						float ey2 = t2*dy+y1;

						// compute the euclidean distance from E to C1 and C2
						float ecdist1 = sqrt(pow((ex1-230),2)+pow((ey1-444),2));
						float ecdist2 = sqrt(pow((ex2-380),2)+pow((ey2-444),2));

						// test if the line intersects the circle 1
						if( ecdist1 < 20 ){
							col1 = 1;
						}
						else if( ecdist1 == 20 ){
							col1 = 1;
						}
				
						// test if the line intersects the circle 2
						if( ecdist2 < 20 ){
							col2 = 1;
						}
						else if( ecdist2 == 20 ){
							col2 = 1;
						}	

						//Same as above slope....Alex
						//std::cout << "Slope " << slope[f] << "\n";
						//++f;	++it2;  Never use either of these variables in this loops...Alex
					}
					
					ld.setLines(lastLines);
					ld.shiftLines(deltax);
					lastLines = ld.getLines();
        
		}
		//if there are lines present use the coordinates stored in the line vector from the current frame
		else{*/
			b = 0; 
			while (it2!=li.end()) {   

					float x1 = (float)(*it2)[0];
					float x2 = (float)(*it2)[2];
					float y1 = (float)(*it2)[1];
					float y2 = (float)(*it2)[3];

					float abdist = sqrt(pow((x2-x1),2)+pow((y2-y1),2));

					// compute the direction vector D from A to B
					float dx = (x2-x1)/abdist;
					float dy = (y2-y1)/abdist;

					//computes slope for comparison between lines to see if they are parallel
					slope[f] = dx/dy;

					// compute the value t of the closest point to the circle 1 and circle 2 center
					float t1 = dx*(230-x1) + dy*(444-y1);
					float t2 = dx*(380-x1) + dy*(444-y1);

					// This is the projection of C on the line from A to B.

					// compute the coordinates of the point E on line and closest to C1 and C2
					float ex1 = t1*dx+x1;
					float ey1 = t1*dy+y1;
					float ex2 = t2*dx+x1;
					float ey2 = t2*dy+y1;

					//std::cout << "point e: (" << ex << "," << ey << ")\n";

					// compute the euclidean distance from E to C1 and C2
					float ecdist1 = sqrt(pow((ex1-230),2)+pow((ey1-444),2));
					float ecdist2 = sqrt(pow((ex2-380),2)+pow((ey2-444),2));

					//std::cout << "distance1: " << ecdist1 << "\n";
					//std::cout << "distance2: " << ecdist2 << "\n";

					// test if the line intersects the circle 1
					if( ecdist1 < 20 ){
						col1 = 1;
					}
					else if( ecdist1 == 20 ){
						col1 = 1;
					}

					// test if the line intersects the circle 2
					if( ecdist2 < 20 ){
						col2 = 1;
					}
					else if( ecdist2 == 20 ){
						col2 = 1;
					}

					futline[b] = slope[f];
					x[e] = x1;
					x[e+1] = x2;
					y[e] = y1;
					y[e+1] = y2;
					e +=2;
					b++;   
					//increment all the counters!
					++f;       
					++it2; 
				}
		//}
		
		
		ld.drawDetectedLines(currentFrame);
		std::cout << "Lines: (" << li_size << ")\n";
		if(li_size != 0){
			lastLines = li;
		}
		


		//changes the color of the displayed circles if the lines intersect
		if(col1 == 1){       
			cv::circle(image,cv::Point(230,444), 20, cv::Scalar(0,0,255,255),-1,8,0);
		}
		else{       
			cv::circle(image,cv::Point(230,444), 20, cv::Scalar(255,0,0,255),-1,8,0);
		}       


		if(col2 == 1){       
			cv::circle(image,cv::Point(380,444), 20, cv::Scalar(0,0,255,255),-1,8,0);
		}
		else{       
			cv::circle(image,cv::Point(380,444), 20, cv::Scalar(255,0,0,255),-1,8,0);
		}



		//cv::namedWindow("Detected Lines (2)");
		cv::imshow("Detected Lanes",currentFrame);
		
		lastFrame = currentFrame;

		/* // Create a Hough accumulator
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

