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

    capture.open(0);

    //capture.open("")

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
        imshow("Original Image",cameraFeed);

        flagMat= myFlagDetector.getFlag(cameraFeed);

        imshow("Flag Image",flagMat);

        waitKey(30);
    }
    
    return 0;
}
