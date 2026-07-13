#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {

  Mat image_red_hsv, image_yellow_hsv, image_green_hsv;
  Mat image_red, image_yellow, image_green;

  image_red = imread("/home/piyush/ADAS/PyDrivingSim/imgs/traffic_signal_red.jfif", IMREAD_COLOR);

  resize(image_red, image_red, Size(400, 400));

  cvtColor(image_red, image_red_hsv, COLOR_BGR2HSV);

  int lh = 0, ls = 100, lv = 100;
  int uh = 10, us = 255, uv = 255; 

  namedWindow("Trackbars", WINDOW_NORMAL);
  createTrackbar("Lower Hue", "Trackbars", &lh, 180);
  createTrackbar("Lower Saturation", "Trackbars", &ls, 255);  
  createTrackbar("Lower Value", "Trackbars", &lv, 255);
  createTrackbar("Upper Hue", "Trackbars", &uh, 180);
  createTrackbar("Upper Saturation", "Trackbars", &us, 255);
  createTrackbar("Upper Value", "Trackbars", &uv, 255);

  while (true) {

    Mat mask;
    inRange(image_red_hsv, Scalar(lh, ls, lv), Scalar(uh, us, uv), mask);
    imshow("Mask", mask);
    imshow ("Original Image", image_red);
    if (waitKey(1) == 27) break;
  }

  return 0;
  
}