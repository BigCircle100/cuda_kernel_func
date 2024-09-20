#ifndef AFFINE_H
#define AFFINE_H

#include "opencv4/opencv2/opencv.hpp"

cv::Mat warp_affine_padding(const cv::Mat& image, const cv::Size& size);

#endif // AFFINE_H