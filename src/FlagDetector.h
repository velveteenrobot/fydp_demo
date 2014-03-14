//FlagDetector.h

#pragma once
#include <opencv2/opencv.hpp>

using namespace std;

class FlagDetector
{
public:
    FlagDetector(void);
    ~FlagDetector(void);

    cv::Mat getFlag(cv::Mat input);
	cv::Mat getBinaryMask(cv::Mat input);
	cv::Mat getBoundingBoxes(cv::Mat binary_mask);
    void removeSmallBlobs(cv::Mat& im, double size);
	void applyBounding(cv::Mat& src, cv::Mat& dest);

private:
    double H_MIN;
    double H_MAX;
    double S_MIN;
    double S_MAX;
    double V_MIN;
    double V_MAX;
    int SMALLEST_AREA;

	cv::Mat calcBoundingBoxes(cv::Mat binary_mask, int r, int g, int b);
	cv::Mat calcThresholdMask(cv::Mat raw_input);
};

// end of FlagDetector.h file
