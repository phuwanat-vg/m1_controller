#pragma once

#include <m1_msgs/M1CpCmd.h>
#include <m1_msgs/M1CpCmdService.h>
#include <m1_msgs/M1CpParams.h>
#include <m1_msgs/M1JogCmd.h>
#include <m1_msgs/M1JogParams.h>
#include <m1_msgs/M1JointCmd.h>
#include <m1_msgs/M1JointCmdService.h>
#include <m1_msgs/M1PtpCmd.h>
#include <m1_msgs/M1PtpCmdService.h>
#include <m1_msgs/M1PtpParams.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>

#include <string>

#include "dobot_m1_interface.hpp"

namespace dobot_m1
{
const float MIN_VEL = 5;
const float MAX_VEL = 100;

const float MIN_ACC = 5;
const float MAX_ACC = 100;

class DobotM1
{
public:
  DobotM1();
  ~DobotM1();

  void ConnectDobot();
  void InitDobot();
  void Homing();

  void PtpCmd(uint8_t mode, float x, float y, float z, float r, bool is_lefthand);
  bool TryPtpCmd(uint8_t mode, float x, float y, float z, float r, bool is_lefthand);

  void CpCmd(uint8_t mode, float x, float y, float z);
  bool TryCpCmd(uint8_t mode, float x, float y, float z);

  void JogParams(float vel, float acc);
  bool TryJogParams(float vel, float acc);

  void IOCmd(uint8_t address, uint8_t level);
  bool TryIOCmd(uint8_t address, uint8_t level);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber ptp_cmd_sub_;
  ros::Subscriber ptp_param_sub_;
  ros::Subscriber cp_cmd_sub_;
  ros::Subscriber cp_param_sub_;
  ros::Subscriber jog_cmd_sub_;
  ros::Subscriber jog_param_sub_;
  ros::Subscriber IO_cmd_sub_;
  void PtpCmdCallback_(const m1_msgs::M1PtpCmd &msg);
  void PtpParamsCallback_(const m1_msgs::M1PtpParams &msg);
  void CpCmdCallback_(const m1_msgs::M1CpCmd &msg);
  void CpParamsCallback_(const m1_msgs::M1CpParams &msg);
  void JogCmdCallback_(const m1_msgs::M1JogCmd &msg);
  void JogParamsCallback_(const m1_msgs::M1JogParams &msg);
  void IOCmdCallback_(const std_msgs::Int16 &msg);
  // timer publisher for joint state
  ros::Timer timer_;
  ros::Publisher joint_pub_;
  void TimerCallback_(const ros::TimerEvent &);

  // services
  ros::ServiceServer ptp_cmd_service_;
  ros::ServiceServer cp_cmd_service_;
  ros::ServiceServer joint_cmd_service_;
  bool PtpCmdServiceCallback_(m1_msgs::M1PtpCmdServiceRequest &req, m1_msgs::M1PtpCmdServiceResponse &res);
  bool CpCmdServiceCallback_(m1_msgs::M1CpCmdServiceRequest &req, m1_msgs::M1CpCmdServiceResponse &res);
  bool JointCmdServiceCallback_(m1_msgs::M1JointCmdServiceRequest &req, m1_msgs::M1JointCmdServiceResponse &res);

  std::string port_;

  float CheckVelocity_(float vel);
  float CheckAcceleration_(float acc);
  void CheckAlarm_();
  bool TryCheckAlarm_();
};
}  // namespace dobot_m1
