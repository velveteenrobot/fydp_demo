//SkinDetector.cpp

//Mat image = cv::imread("red-test.jpg") //in C++ with OpenCV
//flattened array notation:
//image.data[x*nChannels + y*width*d + channel]
//unflattened C++ array notation:
//image.data[y][x][channel]
//http://opencv-srf.blogspot.ro/2010/09/object-detection-using-color-seperation.html

#include "FlagDetector.h"
#include "opencv2/opencv.hpp"

FlagDetector::FlagDetector(void)
{
    //HSV threshold
    H_MIN = 8;
    H_MAX = 173;
    S_MIN = 76.5;
    S_MAX = 255;
    V_MIN = 127.5;
    V_MAX = 255;

    SMALLEST_AREA = 1000;
}

FlagDetector::~FlagDetector(void)
{
}

//this function will return a flag masked image
// IE: This will black out all areas of the original image tha tit does not think is a flag
cv::Mat FlagDetector::getFlag(cv::Mat input)
{
    cv::Mat flag = calcThresholdMask(input);

    cv::cvtColor(flag, flag, CV_GRAY2BGR);
    bitwise_and(flag, input, flag); //Mask only the "red" areas
    
    return flag;
}

// returns a matrix with bounding boxes draw around flag like areas
cv::Mat FlagDetector::getBoundingBoxes(cv::Mat input)
{
	cv::Mat threshold_area = calcThresholdMask(input);
	return calcBoundingBoxes(threshold_area, 255, 0, 0);
}

// returns b/w image with white where it believes the flag is
cv::Mat FlagDetector::getBinaryMask(cv::Mat input)
{
	return calcThresholdMask(input);
}

cv::Mat FlagDetector::calcBoundingBoxes(cv::Mat binary_mask, int r, int g, int b)
{
	cv::Mat bb = binary_mask.clone();
	cv::Mat drawing = cv::Mat::zeros(bb.size(), CV_8UC3);
	vector<vector<cv::Point > > contours;
	vector<cv::Vec4i> hierarchy;

	cv::findContours(bb, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	vector<vector<cv::Point > > contours_poly (contours.size());
	vector<cv::Rect> boundRect (contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]));

		cv::Scalar color = cv::Scalar(b,g,r);
		cv::rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
	}

	return drawing;
}
cv::Mat FlagDetector::calcThresholdMask(cv::Mat raw_input)
{
    cv::Mat flag;
    
    //These may or may not improve deteection
    //cv::blur(input, input, cv::Size(4, 4));
    //cv::imshow("Blurred Lines", input);
    
    //first convert our RGB image to HSV
	cv::GaussianBlur(raw_input,flag,cv::Size(5,5),0,0);
    cv::cvtColor(flag, flag, cv::COLOR_BGR2HSV);

    //uncomment the following line to see the image in HSV Color Space
    //cv::imshow("HSV Color Space", flag);

    //filter the image in HSV color space
    cv::inRange(flag, cv::Scalar(H_MIN, 0, 0), cv::Scalar(H_MAX, 255, 255), flag);
    bitwise_not(flag, flag); //invert the output to detect red
    
    //remove small blobs
    removeSmallBlobs(flag, SMALLEST_AREA);
    
    //Perform morphological closing operation
    cv::Mat structElem = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
    morphologyEx(flag, flag, cv::MORPH_CLOSE, structElem);
    cv::Mat drawing;

    //make blue and g 0, AND it
    cv::cvtColor(flag, flag, CV_GRAY2BGR);
    bitwise_and(flag, raw_input, flag); //Mask only the "red" areas
    cv::cvtColor(flag, flag, cv::COLOR_BGR2HSV); //Back to HSV
    cv::inRange(flag, cv::Scalar(0, S_MIN, V_MIN), cv::Scalar(180, S_MAX, V_MAX), flag); //Apply the saturation and variance masks

	return flag;
}

//Replacement function for bwareaopen
void FlagDetector::removeSmallBlobs(cv::Mat& im, double size)
{
    // Only accept CV_8UC1
    if (im.channels() != 1 || im.type() != CV_8U)
        return;

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(im.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        // Calculate contour area
        double area = cv::contourArea(contours[i]);

        // Remove small objects by drawing the contour with black color
        if (area > 0 && area <= size)
        {
            cv::drawContours(im, contours, i, CV_RGB(0,0,0), -1); //Fill the contour
            cv::drawContours(im, contours, i, CV_RGB(0,0,0), 2); //Draw around the contour
        }
    }
}

//end of FlagDetector.cpp file
