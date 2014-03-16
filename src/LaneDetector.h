//LaneDetector.h

#pragma once
#include <opencv/cv.h>

using namespace std;

class LaneDetector
{
public:
    LaneDetector(void);
    ~LaneDetector(void);

    cv::Mat getLane(cv::Mat input);

private:
	int minimum_pixel_count;

	// Saliency Parameters
	double saliency_threshold_static;
	int saliency_subwindows_static_x;
	int saliency_subwindows_static_y;
	double fisher_threshold_saliency_static;

	int intensity_threshold;

	// HSV parameters
	double v_threshold;
	double s_threshold;
};

// end of LaneDetector.h file
