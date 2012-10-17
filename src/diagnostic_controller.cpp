#include "pr2_diagnostic_controller/diagnostic_controller.h" 
#include <pluginlib/class_list_macros.h>
#include "ethercat_hardware/MotorTraceSample.h"
#include <boost/thread/mutex.hpp>

PLUGINLIB_DECLARE_CLASS(pr2_diagnostic_controller, DiagnosticControllerPlugin, controller::Pr2DiagnosticController, pr2_controller_interface::Controller)

#define BUF_SIZE 4000

namespace controller
{

boost::mutex buffer_lock;

Pr2DiagnosticController::Pr2DiagnosticController()
: robot_(NULL), sample_buf_ptr(NULL), buf_index(0)
{
}

Pr2DiagnosticController::~Pr2DiagnosticController()
{
  delete[] sample_buf_ptr;
}

bool Pr2DiagnosticController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;
  node_ = n;
  sample_buf_ptr = new ethercat_hardware::MotorTraceSample[BUF_SIZE];
  
  if(sample_buf_ptr == NULL)
  {
    ROS_INFO("Could not create buffer for motor trace samples");
    return false;
  }

  srv_ = n.advertiseService("get_diagnostic_data",&Pr2DiagnosticController::getDiagnosticData, this);
  
  return true;
}

void Pr2DiagnosticController::update()
{
  if(robot_ == NULL)
  {
    ROS_ERROR("robot is null");
    return;
  }
  if(robot_->model_ == NULL)
  {
    ROS_ERROR("robot_->model_is null");
    return;
  }
 
  //boost::mutex::scoped_lock lock(buffer_lock, boost::try_lock);
  //if(lock)
  //{
    int num_transmissions = robot_->model_->transmissions_.size();
    ethercat_hardware::MotorTraceSample *temp_sample = NULL;

    for(int i = 0; i < num_transmissions; i++)
    {
      pr2_mechanism_model::Transmission *transmission = robot_->model_->transmissions_[i];
      int num_actuators = transmission->actuator_names_.size();

      for(int j = 0; j < num_actuators; j++)
      {
	temp_sample = robot_->model_->hw_->getData<ethercat_hardware::MotorTraceSample>(transmission->actuator_names_[j]);
	if(temp_sample == NULL)
	{
	  ROS_INFO("no data for actuator %s", transmission->actuator_names_[j].c_str());
	  return;
	}
	else
	{
	  sample_buf_ptr[buf_index % BUF_SIZE].timestamp = temp_sample->timestamp;
	  sample_buf_ptr[buf_index % BUF_SIZE].enabled = temp_sample->enabled;
	  sample_buf_ptr[buf_index % BUF_SIZE].supply_voltage = temp_sample->supply_voltage;
	  sample_buf_ptr[buf_index % BUF_SIZE].measured_motor_voltage = temp_sample->measured_motor_voltage;
	  sample_buf_ptr[buf_index % BUF_SIZE].programmed_pwm = temp_sample->programmed_pwm;
	  sample_buf_ptr[buf_index % BUF_SIZE].executed_current = temp_sample->executed_current;
	  sample_buf_ptr[buf_index % BUF_SIZE].measured_current = temp_sample->measured_current;
	  sample_buf_ptr[buf_index % BUF_SIZE].velocity = temp_sample->velocity;
	  sample_buf_ptr[buf_index % BUF_SIZE].encoder_position = temp_sample->encoder_position;
	  sample_buf_ptr[buf_index % BUF_SIZE].encoder_error_count = temp_sample->encoder_error_count;
	  buf_index++;
	  if(buf_index >= BUF_SIZE)
	  {
	    buf_full = true;
	  }
	}
      }
    }
  //}
  return;

}

void Pr2DiagnosticController::stopping()
{
}

void Pr2DiagnosticController::starting()
{
}

bool Pr2DiagnosticController::getDiagnosticData(pr2_diagnostic_controller::DiagnosticData::Request& req, pr2_diagnostic_controller::DiagnosticData::Response& resp)
{
  if(buf_full)
  {
    for(int i = 0; i < BUF_SIZE; i++)
    {
      pr2_diagnostic_controller::MotorSample temp_sample;
      temp_sample.timestamp = sample_buf_ptr[(i + buf_index) % BUF_SIZE].timestamp;
      temp_sample.enabled = sample_buf_ptr[(i + buf_index) % BUF_SIZE].enabled;
      temp_sample.supply_voltage = sample_buf_ptr[(i + buf_index) % BUF_SIZE].supply_voltage;
      temp_sample.measured_motor_voltage = sample_buf_ptr[(i + buf_index) % BUF_SIZE].measured_motor_voltage;
      temp_sample.programmed_pwm = sample_buf_ptr[(i + buf_index) % BUF_SIZE].programmed_pwm;
      temp_sample.executed_current = sample_buf_ptr[(i + buf_index) % BUF_SIZE].executed_current;
      temp_sample.measured_current = sample_buf_ptr[(i +  buf_index) % BUF_SIZE].measured_current;
      temp_sample.velocity = sample_buf_ptr[(i + buf_index) % BUF_SIZE].velocity;
      temp_sample.encoder_position = sample_buf_ptr[(i + buf_index) % BUF_SIZE].encoder_position;
      temp_sample.encoder_error_count = sample_buf_ptr[(i + buf_index) % BUF_SIZE].encoder_error_count;
      resp.sample_buffer.push_back(temp_sample);
    }
  }
  else
  {
    ROS_INFO("sample buffer is not yet full");
    return false;
  }
  return true;
}

}//end of namespace controller
