#ifndef PR2_DIAGNOSTIC_CONTROLLER_H
#define PR2_DIAGNOSTIC_CONTROLLER_H

#include <pr2_controller_interface/controller.h>
#include <ros/ros.h>
#include <pr2_diagnostic_controller/DiagnosticData.h>
#include <pr2_hardware_interface/hardware_interface.h>

namespace controller
{

class Pr2DiagnosticController: public pr2_controller_interface::Controller
{
public:
  Pr2DiagnosticController();
  ~Pr2DiagnosticController();
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
  bool getDiagnosticData(pr2_diagnostic_controller::DiagnosticData::Request& req, pr2_diagnostic_controller::DiagnosticData::Response& resp);

private:
  pr2_mechanism_model::RobotState* robot_;
  ros::ServiceServer srv_;
  ros::NodeHandle node_;
  ethercat_hardware::MotorTraceSample *sample_buf_ptr;
  int buf_index;
  bool buf_full;
};

}//end of controller namespace
#endif
