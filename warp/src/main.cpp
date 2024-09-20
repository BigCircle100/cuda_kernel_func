#include <iostream>
// #include <opencv4/opencv2/opencv.hpp>

#include "affine.h"


int main(){
  cv::Mat image = cv::imread("pics/water.jpg");
  cv::Size size(640,640);
  cv::Mat output = warp_affine_padding(image, size);
  cv::imwrite("output.png", output);
}