#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

enum class TrafficLightState { RED, YELLOW, GREEN, UNKNOWN };

static string toString(TrafficLightState state) {
  switch (state) {
    case TrafficLightState::RED:    return "RED";
    case TrafficLightState::YELLOW: return "YELLOW";
    case TrafficLightState::GREEN:  return "GREEN";
    default:                        return "UNKNOWN";
  }
}

struct BlobInfo {
  double area = 0.0;
  Rect bounding_box;
};

static BlobInfo bestCircularBlob(const Mat& raw_mask, double min_area,
                                  double min_circularity,
                                  Mat& cleaned_mask_out) {
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  morphologyEx(raw_mask, cleaned_mask_out, MORPH_CLOSE, kernel);
  morphologyEx(cleaned_mask_out, cleaned_mask_out, MORPH_OPEN, kernel);

  vector<vector<Point>> contours;
  findContours(cleaned_mask_out, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  BlobInfo best;
  for (const auto& contour : contours) {
    double area = contourArea(contour);
    if (area < min_area) continue;

    double perimeter = arcLength(contour, true);
    if (perimeter <= 0.0) continue;

    double circularity = 4 * CV_PI * area / (perimeter * perimeter);
    if (circularity < min_circularity) continue;

    if (area > best.area) {
      best.area = area;
      best.bounding_box = boundingRect(contour);
    }
  }

  return best;
}

struct DetectionResult {
  TrafficLightState state = TrafficLightState::UNKNOWN;
  Mat mask;         // cleaned mask of the winning color; empty if UNKNOWN
  Rect bounding_box; // valid only when state != UNKNOWN
};

DetectionResult detectTrafficLightStateDebug(const Mat& bgr_image) {
  DetectionResult result;
  if (bgr_image.empty()) {
    return result;
  }

  Mat hsv;
  cvtColor(bgr_image, hsv, COLOR_BGR2HSV);

  Scalar lower_red(0, 94, 192), upper_red(16, 210, 253);
  Scalar lower_yellow(10, 135, 85), upper_yellow(103, 255, 255);
  Scalar lower_green(58, 76, 44), upper_green(94, 255, 255);

  Mat mask_red_raw, mask_yellow_raw, mask_green_raw;
  inRange(hsv, lower_red, upper_red, mask_red_raw);
  inRange(hsv, lower_yellow, upper_yellow, mask_yellow_raw);
  inRange(hsv, lower_green, upper_green, mask_green_raw);

  double min_area = 0.0005 * bgr_image.rows * bgr_image.cols;
  double min_circularity = 0.6;

  Mat mask_red_clean, mask_yellow_clean, mask_green_clean;
  BlobInfo red = bestCircularBlob(mask_red_raw, min_area, min_circularity, mask_red_clean);
  BlobInfo yellow = bestCircularBlob(mask_yellow_raw, min_area, min_circularity, mask_yellow_clean);
  BlobInfo green = bestCircularBlob(mask_green_raw, min_area, min_circularity, mask_green_clean);

  if (red.area == 0.0 && yellow.area == 0.0 && green.area == 0.0) {
    return result;
  }

  if (red.area >= yellow.area && red.area >= green.area) {
    result.state = TrafficLightState::RED;
    result.mask = mask_red_clean;
    result.bounding_box = red.bounding_box;
  } else if (yellow.area >= red.area && yellow.area >= green.area) {
    result.state = TrafficLightState::YELLOW;
    result.mask = mask_yellow_clean;
    result.bounding_box = yellow.bounding_box;
  } else {
    result.state = TrafficLightState::GREEN;
    result.mask = mask_green_clean;
    result.bounding_box = green.bounding_box;
  }

  return result;
}

TrafficLightState detectTrafficLightState(const Mat& bgr_image) {
  return detectTrafficLightStateDebug(bgr_image).state;
}

int main() {
  struct TestCase { string name; string path; string expected; };
  vector<TestCase> tests = {
      {"red", "/home/piyush/ADAS/PyDrivingSim/imgs/traffic_signal_red.jfif", "RED"},
      {"yellow", "/home/piyush/ADAS/PyDrivingSim/imgs/traffic_signal_yellow.jpg", "YELLOW"},
      {"green", "/home/piyush/ADAS/PyDrivingSim/imgs/traffic_signal_green.jpg", "GREEN"},
  };

  for (const auto& test : tests) {
    Mat image = imread(test.path, IMREAD_COLOR);
    if (image.empty()) {
      cout << test.path << " -> could not load image" << endl;
      continue;
    }

    DetectionResult result = detectTrafficLightStateDebug(image);
    cout << test.path << " -> " << toString(result.state)
         << " (expected " << test.expected << ")" << endl;

    Mat original_resized;
    resize(image, original_resized, Size(500, 500 * image.rows / image.cols));

    if (!result.mask.empty()) {
      Mat boxed = image.clone();
      rectangle(boxed, result.bounding_box, Scalar(0, 255, 255), 8);
      Mat boxed_resized;
      resize(boxed, boxed_resized, original_resized.size());

      imshow(test.name + " - detected " + toString(result.state), boxed_resized);
    } else {
      cout << "  no qualifying blob found, skipping mask/bbox output" << endl;
    }
  }

  cout << "Press any key (with an image window focused) to close all windows..." << endl;
  waitKey(0);
  return 0;
}
