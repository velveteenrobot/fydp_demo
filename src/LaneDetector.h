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

	cv::Mat compute_saliency (cv::Mat lab_image);
	cv::Mat saliency_lanes_static(cv::Mat image, int win_ratio_x, int win_ratio_y);
	cv::Mat cluster_check(cv::Mat subwindow, cv::Mat submap);
	cv::Mat lane_verify(cv::Mat gray_image, cv::Mat lane_mask, int win_ratio_x, int win_ratio_y);
	cv::Mat hsv_verify(cv::Mat image, int v_threshold, int s_threshold);
};

// end of LaneDetector.h file
