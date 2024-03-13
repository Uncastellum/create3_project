// MODIFIED IROBOT_CREATE_NODES__IR_INTENSITY_VECTOR_PUBLISHER_HPP_

#ifndef CREATE3_PROJECT__IR_INTENSITY_PUBLISHER_HPP_
#define CREATE3_PROJECT__IR_INTENSITY_PUBLISHER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/msg/ir_intensity.hpp"
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace create3_project
{

class IrIntensityPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit IrIntensityPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr timer_;

  // Detection publisher
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> publisher_;

  // Subscription
  rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr subscription_;

  // Mutex to protect access to subs_vector_ from different threads
  std::mutex mutex_;

  // Topic to publish IR intensity vector to
  std::string publisher_topic_;

  // Message containing a vector to store IR intensity reasings
  sensor_msgs::msg::LaserScan msg_;

  // Cache of last message for each frame
  // Each IR reading is from a separate publication so there is potential
  // for a race condition where frames could not come in, or come in twice
  // between vector publications.  The cache will keep the most recent
  // reading from each frame and the publication will make a vector of
  // each frame's most recent received reading from the map of values
  std::map<std::string, int> ir_intensities_;
};

}  // namespace create3_project

#endif  // CREATE3_PROJECT__IR_INTENSITY_PUBLISHER_HPP_
