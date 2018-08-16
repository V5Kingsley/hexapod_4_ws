#ifndef GAI_H_
#define GAI_H_

#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <hexapod_msgs/FeetPositions.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "control.h"
 
class Gait
{
public:
  Gait(void);
  void gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet );  //摆动腿和支撑腿切换
  int cycle_period_;
  std::vector<int> cycle_leg_number_;
  bool start_cycle;
  bool is_travelling_ ;
private:
  void cyclePeriod( const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet);  //每条摆动腿和支撑腿一个周期内的步幅控制
  geometry_msgs::Pose2D smooth_base_;
  ros::Time current_time_, last_time_;
  int CYCLE_LENGTH;
  int NUMBER_OF_LEGS;
  double LEG_LIFT_HEIGHT;
  geometry_msgs::Pose2D base;
  int ONE_STEP_LENGTH;
  int ONE_STEP_LENGTH_ORIGIN;
  int LIFT_LENGTH;
  int DROP_LENGTH;
  int MOVE_LENGTH;
  std::vector< std::vector<int> > STEP_GAIT_ORDER;
  std::vector<int> STEP_GAIT_ORDER_1;
  std::vector<int> STEP_GAIT_ORDER_2;
  std::vector<int> STEP_GAIT_ORDER_3;
  std::vector<int> MOVE_GAIT_ORDER;
  
  std::vector< std::vector<int> > THREE_GAIT_ORDER;
  std::vector<int> THREE_GAIT_ORDER_1;
  std::vector<int> THREE_GAIT_ORDER_2;
  
  std::vector< std::vector<int> > FIVE_GAIT_ORDER;
  std::vector<int> FIVE_GAIT_ORDER_1;
  std::vector<int> FIVE_GAIT_ORDER_2;
  std::vector<int> FIVE_GAIT_ORDER_3;
  std::vector<int> FIVE_GAIT_ORDER_4;
  std::vector<int> FIVE_GAIT_ORDER_5;
  std::vector<int> FIVE_GAIT_ORDER_6;
  int gait_order;
  double leg_step[6];
  int step_cout_last_time;
  bool stop_cycle;
  bool stop_active;
  double lift_drop_length;
  double linear_x_max;
  double linear_y_max;
  double angular_z_max;
  int GAIT_NUM;
  int STEP_NUM;
  double support_step;
  double swing_step;
  ros::NodeHandle gn;
}; 

#endif


