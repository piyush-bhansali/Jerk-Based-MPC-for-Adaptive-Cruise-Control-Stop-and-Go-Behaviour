#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "adas_msgs/msg/traffic_light_state_msg.hpp"

using namespace cv;

namespace {

// Must match the trf_light_curr_state / TrafficLightStateMsg convention.
enum class TrafficLightState { UNKNOWN = 0, GREEN = 1, YELLOW = 2, RED = 3 };

std::string toString(TrafficLightState state) {
  switch (state) {
    case TrafficLightState::GREEN:  return "GREEN";
    case TrafficLightState::YELLOW: return "YELLOW";
    case TrafficLightState::RED:    return "RED";
    default:                        return "UNKNOWN";
  }
}

struct BlobInfo {
  double area = 0.0;
  Rect bounding_box;
};

BlobInfo bestCircularBlob(const Mat& raw_mask, double min_area, double min_circularity) {
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  Mat cleaned;
  morphologyEx(raw_mask, cleaned, MORPH_CLOSE, kernel);
  morphologyEx(cleaned, cleaned, MORPH_OPEN, kernel);

  std::vector<std::vector<Point>> contours;
  findContours(cleaned, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

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
  Rect bounding_box;  // valid only when state != UNKNOWN
};

DetectionResult detectTrafficLightState(const Mat& bgr_image) {
  DetectionResult result;
  if (bgr_image.empty()) {
    return result;
  }

  Mat hsv;
  cvtColor(bgr_image, hsv, COLOR_BGR2HSV);

  Scalar lower_red(0, 94, 192), upper_red(16, 210, 253);
  Scalar lower_yellow(10, 135, 85), upper_yellow(103, 255, 255);
  Scalar lower_green(58, 76, 44), upper_green(94, 255, 255);

  Mat mask_red, mask_yellow, mask_green;
  inRange(hsv, lower_red, upper_red, mask_red);
  inRange(hsv, lower_yellow, upper_yellow, mask_yellow);
  inRange(hsv, lower_green, upper_green, mask_green);

  double min_area = 0.0005 * bgr_image.rows * bgr_image.cols;
  double min_circularity = 0.6;

  BlobInfo red    = bestCircularBlob(mask_red, min_area, min_circularity);
  BlobInfo yellow = bestCircularBlob(mask_yellow, min_area, min_circularity);
  BlobInfo green  = bestCircularBlob(mask_green, min_area, min_circularity);

  if (red.area == 0.0 && yellow.area == 0.0 && green.area == 0.0) {
    return result;
  }

  if (red.area >= yellow.area && red.area >= green.area) {
    result.state = TrafficLightState::RED;
    result.bounding_box = red.bounding_box;
  } else if (yellow.area >= red.area && yellow.area >= green.area) {
    result.state = TrafficLightState::YELLOW;
    result.bounding_box = yellow.bounding_box;
  } else {
    result.state = TrafficLightState::GREEN;
    result.bounding_box = green.bounding_box;
  }

  return result;
}

}  // namespace

class TrafficLightDetectorNode : public rclcpp::Node {
public:
  TrafficLightDetectorNode() : Node("traffic_light_detector") {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/traffic_light_image", 10,
      std::bind(&TrafficLightDetectorNode::image_callback, this, std::placeholders::_1));

    state_pub_ = create_publisher<adas_msgs::msg::TrafficLightStateMsg>("/traffic_light_state", 10);
  }

  ~TrafficLightDetectorNode() override {
    if (has_window_) {
      destroyWindow(window_name_);
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<adas_msgs::msg::TrafficLightStateMsg>::SharedPtr state_pub_;

  bool has_window_ = false;
  std::string window_name_;
  TrafficLightState last_state_ = TrafficLightState::UNKNOWN;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    DetectionResult result = detectTrafficLightState(cv_ptr->image);

    adas_msgs::msg::TrafficLightStateMsg out;
    out.state = static_cast<int32_t>(result.state);
    state_pub_->publish(out);

    // Open a fresh window whenever the detected color changes; keep reusing it
    // for as long as the same image (same light color) keeps being published.
    if (!has_window_ || result.state != last_state_) {
      if (has_window_) {
        destroyWindow(window_name_);
      }
      window_name_ = "Traffic Light Detector - " + toString(result.state);
      namedWindow(window_name_, WINDOW_NORMAL);
      has_window_ = true;
      last_state_ = result.state;
    }

    Mat display = cv_ptr->image.clone();
    if (result.state != TrafficLightState::UNKNOWN) {
      rectangle(display, result.bounding_box, Scalar(0, 255, 255), 8);
    }

    Mat resized;
    resize(display, resized, Size(500, std::max(1, 500 * display.rows / display.cols)));
    imshow(window_name_, resized);
    waitKey(1);
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficLightDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
