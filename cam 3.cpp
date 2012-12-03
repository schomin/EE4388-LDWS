

#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
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

		std::cout << "Lines detected: " << lines.size() << "\n";

		//Define stuff here
		//slope is used to determine if two lines are parallel
		float slope[lines.size()];  
		//futline is used to store the slope of past lines in order to compare them between each other
		float futline[lines.size()];
		//the x coordinates of stored lines 
		float x[2*(lines.size())];  
		//the y coordinates of stored lines
		float y[2*(lines.size())];  
		//incrementer for the slope, x/y, and futline arrays
		int f = 0; 
		int e = 0; 
		int b = 0; 
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


		// Display the detected line image
		//cv::namedWindow("Detected Lines with Hough");
		//cv::imshow("Detected Lines (2)",result);

		// Create LineFinder instance
		LineFinder ld;

		// Set probabilistic Hough parameters  ---- we can fiddle with this later in order to improve accuracy (2nd semester) ...Alex
		ld.setLineLengthAndGap(100,20);
		ld.setMinVote(80);

		// Detect lines
		std::vector<cv::Vec4i> li= ld.findLines(contours);
		ld.drawDetectedLines(image);

		// eliminate inconsistent lines   ----maybe use the slopes found within the while loops to eliminate certain lines?? ...Alex
		ld.removeLinesOfInconsistentOrientations(ed.getOrientation(),0.4,0.1);

		std::vector<cv::Vec4i>::const_iterator it2= li.begin();

		//if there are no lines use the coordinates stored in the x and y arrays
		if(it2==li.end()){		
				//changed this to a for loop instead of a while loop so we could use the multiple parallel lines stored in the futline and x/y arrays...Alex
				for (int g = 0; g < b; g++) { //yet another new integer for counting purposes...Alex
						//uses the arrays with stored past values instead of the line vector
						float x1 = x[g];
						float x2 = x[g+1]; 
						float y1 = y[g];
						float y2 = y[g+1];

						//std::cout << "point 1: (" << x1 << "," << y1 << ")\n";
						//std::cout << "point 2: (" << x2 << "," << y2 << ")\n";

						float abdist = sqrt(pow((x2-x1),2)+pow((y2-y1),2));

						//std::cout << "abdist: (" << abdist << ")\n";

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

						//Same as above slope....Alex
						//std::cout << "Slope " << slope[f] << "\n";
						//++f;	++it2;  Never use either of these variables in this loops...Alex
						}
        
		}
		//if there are lines present use the coordinates stored in the line vector from the current frame
		else{
			while (it2!=li.end()) {   

					float x1 = (float)(*it2)[0];
					float x2 = (float)(*it2)[2];
					float y1 = (float)(*it2)[1];
					float y2 = (float)(*it2)[3];

					//std::cout << "point 1: (" << x1 << "," << y1 << ")\n";
					//std::cout << "point 2: (" << x2 << "," << y2 << ")\n";

					float abdist = sqrt(pow((x2-x1),2)+pow((y2-y1),2));

					//std::cout << "abdist: (" << abdist << ")\n";

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
					//std::cout << "Slope " << slope[f] << "\n";

					//the initial slope and points should always be included as a comparison...Alex
					if(f == 0){
						//store the coordinates of x, y, and the slope
						futline[0] = slope[f];
						x[0] = x1;	x[1] = x2;	y[0] = y1;	y[1] = y2;	e +=2;	b++;
					}
					else (f != 0){
						//store the matching slopes into the arrays to use if there are no lines in the next frame
						if(futline[b-1] == slope[f]){
							//if the slopes are equal store the coordinates of x and y 
							futline[b] = slope[f];
							x[e] = x1;
							x[e+1] = x2;
							y[e] = y1;
							y[e+1] = y2;
							e +=2;
							b++;                               
						}
					}
				//increment all the counters!
				++f;       
				++it2; 
				}
		}

		/*we initially had this outside of the while loop but I moved it inside to simplify things a bit and take away the need for some many loops...Alex

		int b = 0; //yet another counter...I moved this outside the loop so it wouldnt be reset to 0 every iteration...Alex
		//store the matching slopes (x,y) into the arrays
				for(int k = f; k >= 0; k--)
				{
						for(int j = k; j > 0; j--)
						{
								if(futline[b-1] == slope[j]){ 

		//i ended up changing this in the actual code b/c if you check against every stored slope in the array you will find matching slopes multiple times
						
									//if the slopes are equal store the coordinates of x and y 
									futline[b] = slope[f];
									x[e] = x1;
									x[e+1] = x2;
									y[e] = y1;
									y[e+1] = y2;
									e +=2;
									b++;    
                                
								}
						}
				}
		*/


		// Display the detected line image
		image= frame;
		//ld.drawDetectedLines(image);


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
		cv::imshow("Detected Lines (2)",image);

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
