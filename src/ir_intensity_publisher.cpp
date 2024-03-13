// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "create3_project/ir_intensity_publisher.hpp"

#include <vector>
#include <string>
#include <iostream>

#include "rclcpp_components/register_node_macro.hpp"

#define fmin(a, b) ((a) < (b) ? (a) : (b))

namespace create3_project
{

/*
The arrangement of the sensors is defined in the following schema of the top
view of the robot:
                        Front view
                            |
                            V

                      front_center_left
              front_left            front_center_right
          left                                    front_right
side_left                                                   right
*/

int str2intSchema(std::string sch) {
  if (sch == "ir_intensity_side_left") {
    return 0;
  } else if (sch == "ir_intensity_left") {
    return 1;
  } else if (sch == "ir_intensity_front_left") {
    return 2;
  } else if (sch == "ir_intensity_front_center_left") {
    return 3;
  } else if ( sch == "ir_intensity_front_center_right") {
    return 4;
  } else if (sch == "ir_intensity_front_right") {
    return 5;
  } else { // case "ir_intensity_right"
    return 6;
  }
}

IrIntensityPublisher::IrIntensityPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("ir_intensity_lasermsg_node", options)
{
  RCLCPP_INFO_STREAM(get_logger(), "Hola");
  // Topic parameter to publish IR intensity vector to
  publisher_topic_ =
    this->declare_parameter("publisher_topic", "scan");

  // Publish rate parameter in Hz
  const double publish_rate =
    this->declare_parameter("publish_rate", 10.0);

  publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
    publisher_topic_, rclcpp::SensorDataQoS().reliable());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << publisher_topic_);

  //RCLCPP_INFO_STREAM(get_logger(), "pb: " << publish_rate);

  // CONSTANT PARAMETERS:
  // 18.5 degrees == 18.5 * (3.14159265359) / 180 == 0.323 rads
  // 55.5 degrees = 18.5 * 3 = 0.969 rads
  this->msg_.angle_min = -(0.323 * 3);
  this->msg_.angle_max = (0.323 * 3);
  this->msg_.angle_increment = 0.323;
  // Time?
  this->msg_.time_increment = 0.5;
  this->msg_.scan_time = 0.5;
  // -- close, ++ far
  this->msg_.range_min = 0.028;
  this->msg_.range_max = 0.35;

  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(std::chrono::duration<double>(1 / publish_rate)), [this]() {
      std::lock_guard<std::mutex> lock{this->mutex_};

      // Set header timestamp.
      this->msg_.header.stamp = now();

      float intensitiesAUX[7];
      for (const auto & ir_msgs : this->ir_intensities_) {
        intensitiesAUX[str2intSchema(ir_msgs.first)] = ir_msgs.second;
      }

      // Debug
      /*std::ostringstream oss;
      oss << std::setw(6) <<std::fixed<<std::setprecision(3)<< intensitiesAUX[0]
          << std::setw(6) <<std::fixed<<std::setprecision(3)<< intensitiesAUX[1] << std::setw(6) <<std::fixed<<std::setprecision(3)<< intensitiesAUX[2]
          << std::setw(6) <<std::fixed<<std::setprecision(3)<< intensitiesAUX[3] << std::setw(6) <<std::fixed<<std::setprecision(3)<< intensitiesAUX[4]
          << std::setw(6) <<std::fixed<<std::setprecision(3)<< intensitiesAUX[5] << std::setw(6) <<std::fixed<<std::setprecision(3)<< intensitiesAUX[6];
      oss << std::setw(10) <<std::fixed<<std::setprecision(3)<< intensitiesAUX[0];
      RCLCPP_INFO_STREAM(get_logger(), oss.str().c_str());*/

      this->msg_.ranges.clear();
      this->msg_.ranges.reserve(7);
      for (const float ir : intensitiesAUX) {
        float calc = 5.0f/sqrt(10*ir+1);
        this->msg_.ranges.emplace_back(calc);
      }

      // Publish detected vector.
      this->publisher_->publish(this->msg_);
    });
  // Set header frame_id.
  this->msg_.header.frame_id = "laser_frame";

  subscription_ = create_subscription<irobot_create_msgs::msg::IrIntensityVector>(
    "/ir_intensity", rclcpp::SensorDataQoS(),
    [this](const irobot_create_msgs::msg::IrIntensityVector::SharedPtr msg) {
      std::lock_guard<std::mutex> lock{this->mutex_};
      for (const auto & frame : msg->readings) {
        this->ir_intensities_[frame.header.frame_id] = frame.value;
      }
    });
}

}  // namespace create3_project

RCLCPP_COMPONENTS_REGISTER_NODE(create3_project::IrIntensityPublisher)
