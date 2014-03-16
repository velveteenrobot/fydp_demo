#include "LaneDetector.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cmath>
#include "cvblob.h"

LaneDetector::LaneDetector(void)
{
    minimum_pixel_count = 400;

    saliency_threshold_static = 0.4;
	saliency_subwindows_static_x = 4;
	saliency_subwindows_static_y = 4;
	fisher_threshold_saliency_static = 0.001;

	intensity_threshold = 100;

    v_threshold = 0.7;
	s_threshold = 0.1;
}

LaneDetector::~LaneDetector(void)
{
}

inline double round( double d )
{
    return floor( d + 0.5 );
}

cv::Mat LaneDetector::compute_saliency (cv::Mat lab_image)
{
	lab_image.convertTo(lab_image, CV_32F);
	lab_image = lab_image/255;
	cv::Scalar m = mean(lab_image);
	cv::vector<cv::Mat> channels(3);
    split(lab_image, channels);
	cv::Mat saliency = channels[0] + channels[1] + channels[2] - m[0] - m[1] - m[2];
	cv::threshold(saliency, saliency, 0.0, 1.0, 3);
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;

    minMaxLoc( saliency, &minVal, &maxVal, &minLoc, &maxLoc );

	return saliency / maxVal;
}

cv::Mat LaneDetector::saliency_lanes_static(cv::Mat image, int win_ratio_x, int win_ratio_y)
{
    cv::Size s = image.size();
    int rows = s.height;
    int cols = s.width;

    cv::Mat saliency_map_out = cv::Mat(rows, cols, CV_64F, 0.0);
    cv::Mat saliency_bin_out = cv::Mat(rows, cols, CV_64F, 0.0);

    int nominal_win_size_x = round(rows/win_ratio_x);
    int nominal_win_size_y = round(cols/win_ratio_y);

    cv::Mat lab_image;
    cvtColor( image, lab_image, CV_BGR2Lab );

    for (int j = 1; j <= win_ratio_x; j++)
        {
		for (int i = 1; i <= win_ratio_y; i++)
			{
            int top = (i - 1)*nominal_win_size_x;
            int bottom = std::min(i*nominal_win_size_x, rows);
            int left = (j - 1)*nominal_win_size_y;
            int right = std::min(j*nominal_win_size_y, cols);

            cv::Mat subimage = lab_image.colRange(left, right).rowRange(top, bottom);
            cv::Mat submap_out = compute_saliency(subimage);

            cv::Mat sub_bin_out = submap_out > saliency_threshold_static;

			submap_out = submap_out * 255;
			sub_bin_out = sub_bin_out * 255;
			submap_out.copyTo(saliency_map_out.colRange(left, right).rowRange(top, bottom));
			sub_bin_out.copyTo(saliency_bin_out.colRange(left, right).rowRange(top, bottom));
		}
    }
	return saliency_bin_out;
}

cv::Mat LaneDetector::cluster_check(cv::Mat subwindow, cv::Mat submap)
{
    cv::Size s = subwindow.size();
    int rows = s.height;
    int cols = s.width;
	
    cv::Mat verified_map_out = cv::Mat::zeros(rows, cols, CV_32F);
	cv::Mat temp;
	submap.convertTo(temp, CV_8U);
	IplImage iplsubmap = temp;
	IplImage *labelImg = cvCreateImage(s, IPL_DEPTH_LABEL, 1);
	cvb::CvBlobs blobs;
	cvb::cvLabel(&iplsubmap, labelImg, blobs);

	for (cvb::CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
	{
		if (it->second->area > minimum_pixel_count)
		{
			IplImage *individualBlobImg = cvCreateImage(s, 8, 3);
			cvSet(individualBlobImg, CV_RGB(0, 0, 0));
			cv::cvtColor(temp, temp, CV_GRAY2BGR);
			iplsubmap = temp;
			cvb::cvRenderBlobs(labelImg, *it->second, &iplsubmap, individualBlobImg, CV_BLOB_RENDER_COLOR);
			cv::Mat output(individualBlobImg);			
			cv::vector<cv::Mat> channels(3);
			split(output, channels);
			cv::vector<cv::Mat> channells(3);
			split(verified_map_out, channells);
			cv::Scalar mean_cluster = sum(subwindow.mul(channels[0])) / (cv::Scalar)(it->second->area);
			cv::Scalar mean_else = mean(subwindow);
			channels[0].convertTo(temp, CV_32F);
			if (mean_cluster[0] > mean_else[0])
			{
				// TO-DO: Variance stuff
				cv::bitwise_or(verified_map_out, temp, verified_map_out);
			}
		}
	}

	return verified_map_out;
}

cv::Mat LaneDetector::lane_verify(cv::Mat gray_image, cv::Mat lane_mask, int win_ratio_x, int win_ratio_y)
{
    cv::Size s = gray_image.size();
    int rows = s.height;
    int cols = s.width;

    cv::Mat lane_map = cv::Mat(rows, cols, CV_64F, 0.0);
    cv::Mat lane_map_out = cv::Mat(rows, cols, CV_64F, 0.0);
	
    int nominal_win_size_x = round(rows/win_ratio_x);
    int nominal_win_size_y = round(cols/win_ratio_y);

    for (int j = 1; j <= win_ratio_x; j++)
        {
		for (int i = 1; i <= win_ratio_y; i++)
			{
            int top = (i - 1)*nominal_win_size_x;
            int bottom = std::min(i*nominal_win_size_x, rows);
            int left = (j - 1)*nominal_win_size_y;
            int right = std::min(j*nominal_win_size_y, cols);

            cv::Mat subwindow = gray_image.colRange(left, right).rowRange(top, bottom);
            cv::Mat submap = lane_mask.colRange(left, right).rowRange(top, bottom);
            cv::Mat submap_out = cluster_check(subwindow, submap);
			submap_out.copyTo(lane_map.colRange(left, right).rowRange(top, bottom));
        }
    }

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    morphologyEx(lane_map, lane_map, cv::MORPH_CLOSE, element);

	cv::Mat temp;
	lane_map.convertTo(temp, CV_8U);
	IplImage ipllanemap = temp;
	IplImage *labelImg = cvCreateImage(s, IPL_DEPTH_LABEL, 1);
	cvb::CvBlobs blobs;
	cvb::cvLabel(&ipllanemap, labelImg, blobs);
	cvb::cvFilterByArea(blobs, 5*minimum_pixel_count, 1000*minimum_pixel_count);

	IplImage *black = cvCreateImage(s, 8, 3);
	cvSet(black, CV_RGB(0, 0, 0));
	cvRenderBlobsReally(labelImg, blobs, &ipllanemap, black, CV_BLOB_RENDER_COLOR);
	cv::Mat output(black);
	cv::cvtColor(output, output, CV_BGR2GRAY, 1);

	return output;
}

cv::Mat LaneDetector::hsv_verify(cv::Mat image, int v_threshold, int s_threshold)
{
	cv::Size s = image.size();
    cv::Mat hsv_image;
    cvtColor( image, hsv_image, CV_BGR2HSV );
    cv::vector<cv::Mat> channels(3);
    split(hsv_image, channels);

	cv::Mat value_mask;
	cv::Mat saturation_mask;
	cv::threshold(channels[1], value_mask, 50, 255, 1);
	cv::threshold(channels[2], saturation_mask, 150, 255, 0);
	return value_mask.mul(saturation_mask);
}

cv::Mat LaneDetector::getLane(cv::Mat image)
{
	cv::Mat gray_image;
	cv::Mat image_blurred;
	cv::GaussianBlur(image, image_blurred, cv::Size(7, 7), 0, 0);
    cvtColor(image_blurred, gray_image, CV_BGR2GRAY );

    cv::Mat saliency_initial_lane_map_static = saliency_lanes_static(image_blurred, saliency_subwindows_static_x, saliency_subwindows_static_y);
	cv::Mat saliency_verified_lane_map_static = lane_verify(gray_image, saliency_initial_lane_map_static, saliency_subwindows_static_x, saliency_subwindows_static_y);
	
    cv::Mat hsv_mask = hsv_verify(image, v_threshold, s_threshold);
	cv::Mat hsv_lane_map;
	hsv_lane_map = saliency_verified_lane_map_static.mul(hsv_mask);

	cv::Mat yellow = cvCreateImage(image.size(), 8, 3);
	yellow = cv::Scalar(0, 255, 255);

    cv::Mat lanes_out = image;
	yellow.copyTo(lanes_out, hsv_lane_map);
	
    return lanes_out;
}

//end of LaneDetector.cpp file
