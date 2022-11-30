#ifndef CNR_OPEN_LOOP_POSVEL_CONTROLLER__CNR_OPEN_LOOP_POSVEL2_CONTROLLER__H
#define CNR_OPEN_LOOP_POSVEL_CONTROLLER__CNR_OPEN_LOOP_POSVEL2_CONTROLLER__H


#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <thread>
#include <mutex>
#include <fstream>
#include <boost/graph/graph_concepts.hpp>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <subscription_notifier/subscription_notifier.h>

// #include <cnr_hardware_interface/posvel_command_interface.h>

namespace cnr
{
namespace control
{

/**
 * @brief The OpenLoopPosVel2Controller class
 */
class OpenLoopPosVel2Controller :
  public cnr::control::JointCommandController<hardware_interface::PosVelJointHandle,
                                                            hardware_interface::PosVelJointInterface>
{
public:
  OpenLoopPosVel2Controller(void):logFile("pv2_output.csv",std::ofstream::app){};
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:
  std::ofstream logFile;

  std::string m_setpoint_topic_name;
  bool        m_configured;

  void callback(const sensor_msgs::JointStateConstPtr& msg);
  bool extractJoint(const sensor_msgs::JointState& msg);

  const std::string   SP_TOPIC_ID = "sp";
};

}  // namespace control
}  // namespace cnr

# endif  // CNR_OPEN_LOOP_POSVEL_CONTROLLER__CNR_OPEN_LOOP_POSVEL2_CONTROLLER__H
