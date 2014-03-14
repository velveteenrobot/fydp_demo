//Here image will be captured from the webcam and show the masked image. 
//If you want to masked from video file then you have change the 
//capture.open(0);
//to capture.open("filenameWithPath.extention");

#include <opencv2/opencv.hpp>
#include "FlagDetector.h"
using namespace std;
using namespace cv;

int main()
{

    VideoCapture capture;
    //open capture object at location zero (default location for webcam)

    //capture.open(0);

   capture.open("C:\\Users\\andylee\\Downloads\\2014-03-13 21.04.32.mp4");

    //some video file
    //capture.open("red-test.jpg")

    //set height and width of capture frame
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    Mat cameraFeed;

    FlagDetector myFlagDetector;

    Mat flagMat;

    //start an infinite loop where webcam feed is copied to cameraFeed matrix
    //all of our operations will be performed within this loop
    while(1){

        //store image to matrix
        capture.read(cameraFeed);

        //show the current image

		flagMat= myFlagDetector.getBoundingBoxes(cameraFeed);
		imshow("Original Image",cameraFeed + flagMat);

        //imshow("Flag Image",flagMat);

        waitKey(30);
    }
    
    return 0;
}
