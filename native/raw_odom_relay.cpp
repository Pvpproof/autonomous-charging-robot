#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
class RawRelay: public rclcpp::Node {
 public:
  RawRelay(): Node("charging_raw_odom_relay") {
    std::ifstream f("/proc/sys/kernel/random/uuid");f>>instance_;if(instance_.empty())throw std::runtime_error("instance unavailable");
    pub_=create_publisher<std_msgs::msg::String>("/charging_control/raw_odometry",10);
    sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m,const rclcpp::MessageInfo & info){emit(m,info);});
  }
 private:
  void emit(nav_msgs::msg::Odometry::ConstSharedPtr m,const rclcpp::MessageInfo& info) {
    std_msgs::msg::String out;auto q=m->pose.pose.orientation;auto p=m->pose.pose.position;auto v=m->twist.twist;
    double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(m->header.frame_id!="odom_combined"||m->child_frame_id!="base_footprint"||!std::isfinite(norm)||std::abs(norm-1.)>.02||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z)) {out.data="{\"error\":\"invalid raw odometry frame/quaternion/value\"}";pub_->publish(out);return;}
    std::ostringstream gid;gid<<std::hex<<std::setfill('0');for(int i=0;i<16;++i)gid<<std::setw(2)<<int(info.get_rmw_message_info().publisher_gid.data[i]);
    double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    std::ostringstream s;s<<std::setprecision(17)<<"{\"relay_instance\":\""<<instance_<<"\",\"relay_sequence\":"<<++sequence_<<",\"publisher_gid\":\""<<gid.str()<<"\",\"source_stamp\":"<<double(m->header.stamp.sec)+double(m->header.stamp.nanosec)*1e-9<<",\"relay_received_ros\":"<<now().seconds()<<",\"pose\":["<<p.x<<","<<p.y<<","<<yaw<<"],\"velocity\":["<<v.linear.x<<","<<v.linear.y<<","<<v.angular.z<<"],\"frame\":\"odom_combined\",\"child\":\"base_footprint\"}";
    out.data=s.str();pub_->publish(out);
  }
  std::string instance_;uint64_t sequence_=0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<RawRelay>());rclcpp::shutdown();return 0;}
