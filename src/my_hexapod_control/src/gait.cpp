/*****************************************
 *                   六足控制算法四脚步态规划程序          *
 *                   Copyright (c) V5_Lab, 2018                  *
 *                   Author:                  Kingsley                  *
 *                   Version number:  0.00                         *
 *                   Date:                                                      *
 * ***************************************/

#include "gait.h"
static const double PI=3.141592653;

Gait::Gait( void )
{
 // ros::param::get( "CYCLE_LENGTH", CYCLE_LENGTH );
  ros::param::get( "LEG_LIFT_HEIGHT", LEG_LIFT_HEIGHT );
  ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
  ros::param::get("ONE_STEP_LENGTH", ONE_STEP_LENGTH_ORIGIN);
  ros::param::get("STEP_GAIT_ORDER_1", STEP_GAIT_ORDER_1);
  ros::param::get("STEP_GAIT_ORDER_2", STEP_GAIT_ORDER_2);
  ros::param::get("STEP_GAIT_ORDER_3", STEP_GAIT_ORDER_3);
  ros::param::get("LINEAR_X_MAX", linear_x_max);
  ros::param::get("LINEAR_Y_MAX", linear_y_max);
  ros::param::get("ANGULAR_Z_MAX", angular_z_max);
  STEP_GAIT_ORDER.resize(3);
  STEP_GAIT_ORDER[0] = STEP_GAIT_ORDER_1;
  STEP_GAIT_ORDER[1] = STEP_GAIT_ORDER_2;
  STEP_GAIT_ORDER[2] = STEP_GAIT_ORDER_3;
  ONE_STEP_LENGTH = ONE_STEP_LENGTH_ORIGIN;
  
  cycle_period_=1;
  gait_order = 0;
  leg_step[6]={0};
  stop_cycle = 0;
  stop_active = 0;
  lift_drop_length = 0.3;
}

//每条摆动腿和支撑腿一个周期内的步幅控制
void Gait::cyclePeriod( const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet )
{
//   double period_distance, period_height;
//   period_distance = -cos( cycle_period_*PI/CYCLE_LENGTH );  //每条腿PI/CYCLE_LENGTH时间的步幅
//   period_height = sin( cycle_period_*PI/CYCLE_LENGTH ); //摆动腿PI/CYCLE_LENGTH时间抬起的高度
  
  int step_cout;
  if(gait_order == 0)
  {
    step_cout = 2;
  }
  else
  {
    step_cout = gait_order - 1;
  }
  if (MOVE_GAIT_ORDER.empty())
  {
    ROS_FATAL("Please set the speed to 0 and restart.");
    ros::shutdown();
    return;
  }
  
//   std::cout<<step_cout<<" : ";
//   std::vector<int>::iterator pd;
//   for(pd = MOVE_GAIT_ORDER.begin(); pd != MOVE_GAIT_ORDER.end(); pd++)
//   {
//     std::cout<<*pd<<" ";
//   }
//   std::cout<<std::endl;
  bool step_change = std::abs(step_cout - step_cout_last_time);
  step_cout_last_time = step_cout;

  //摆动腿和支撑腿歩幅控制
  for( int leg_index=0; leg_index<NUMBER_OF_LEGS; leg_index++ )
  {
    //摆动腿
    if( MOVE_GAIT_ORDER[leg_index]==1 )
    {
      double period_distance, period_height;
      int step_period = cycle_period_-step_cout*ONE_STEP_LENGTH;
      if(step_period <= LIFT_LENGTH)
      {
	period_distance = leg_step[leg_index] - 2.0/3.0;
	period_height = - 0.5 * cos( M_PI * step_period / LIFT_LENGTH) + 0.5;
	ROS_INFO("lift, step_period: %d", step_period);
      }
      else if(step_period >= (ONE_STEP_LENGTH-DROP_LENGTH))
      {
	period_distance = leg_step[leg_index];
	period_height = 0.5 * cos( M_PI * (step_period - (LIFT_LENGTH+MOVE_LENGTH)) / DROP_LENGTH ) + 0.5;
	ROS_INFO("drop, step_period: %d", step_period);
      }
      else
      {
	period_distance = - (1.0/3.0) * cos( M_PI * (step_period-LIFT_LENGTH) / MOVE_LENGTH ) + (leg_step[leg_index]-2.0/3.0+1.0/3.0);
	period_height = 1;
	ROS_INFO("move, step_period: %d, period_distance: %f", step_period, period_distance);
      }
      feet->foot[leg_index].position.x = base.x * period_distance;
      feet->foot[leg_index].position.y = base.y * period_distance;
      feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height ;
      feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
    }
    //支撑腿
    if( MOVE_GAIT_ORDER[leg_index]==0 )
    {
      double period_distance;
      int step_period = cycle_period_-step_cout*ONE_STEP_LENGTH;
      if(step_period <= LIFT_LENGTH)
      {
	period_distance = leg_step[leg_index] + 1.0/3.0;
      }
      else if(step_period >= (ONE_STEP_LENGTH-DROP_LENGTH))
      {
	period_distance = leg_step[leg_index];
      }
      else
      {
	period_distance = 1.0/6.0 * cos(M_PI * (step_period-LIFT_LENGTH) / MOVE_LENGTH) + (1.0/6.0+leg_step[leg_index]);
      }  
      feet->foot[leg_index].position.x = base.x * period_distance;
      feet->foot[leg_index].position.y = base.y * period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
    }
  }
  
  if(stop_cycle == 1 && cycle_period_ == CYCLE_LENGTH)
  {
    stop_cycle = 0;
  }
  

}
 
//摆动腿和支撑腿切换
void Gait::gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet )
{
  
  if( (base.x - cmd_vel.linear.x) != 0 || (base.y - cmd_vel.linear.y) != 0 || (base.theta - cmd_vel.angular.z) != 0)
  {
    
    if(cmd_vel.linear.x == 0 && cmd_vel.linear.y == 0 && cmd_vel.angular.z == 0 && stop_active == 0)
    {
      stop_cycle = 1;
      stop_active = 1;
    }
    
    if(stop_cycle == 0)
    {
      stop_active = 0;
      base.x = cmd_vel.linear.x ; 
      base.y = cmd_vel.linear.y ;
      base.theta = cmd_vel.angular.z ;
    }
    
    //周期调整
  LIFT_LENGTH = lift_drop_length * ONE_STEP_LENGTH_ORIGIN;
  DROP_LENGTH = lift_drop_length * ONE_STEP_LENGTH_ORIGIN;
  double x_cycle = std::abs(base.x)/linear_x_max;
  double y_cycle = std::abs(base.y)/linear_y_max;
  double z_cycle = std::abs(base.theta)/angular_z_max;
  double max_cycle = (x_cycle>y_cycle?x_cycle:y_cycle) > z_cycle ? (x_cycle>y_cycle?x_cycle:y_cycle) : z_cycle;
  MOVE_LENGTH = (1-2*lift_drop_length) * ONE_STEP_LENGTH_ORIGIN * max_cycle;
  ONE_STEP_LENGTH = LIFT_LENGTH + MOVE_LENGTH + DROP_LENGTH;
  CYCLE_LENGTH = ONE_STEP_LENGTH * 3;
  
  smooth_base_.x = base.x;
  smooth_base_.y = base.y;
  smooth_base_.theta = base.theta;
  
  }

    
    // Check to see if we are actually travelling
    if( ( std::abs( smooth_base_.y ) > 0.001 ) || // 1 mm
        ( std::abs( smooth_base_.x ) > 0.001 ) || // 1 mm
        ( std::abs( smooth_base_.theta ) > 0.00436332313 ) ) // 0.25 degree
    {
        is_travelling_ = true;
    }
    else
    {
        is_travelling_ = false;  
    }
    if ( is_travelling_ == true )
    {
      //给下一个period/CYCLE_LENGTH足端歩幅
      cyclePeriod( smooth_base_, feet);
      cycle_period_++;
    }
    
    //一个周期结束后从1计数
    if( cycle_period_==(CYCLE_LENGTH+1) )
    {
      cycle_period_=1;
    } 
    
    /*每个抬腿周期（1/3周期）后更换摆动腿组和支撑腿组*/
    if(cycle_period_ == (gait_order*ONE_STEP_LENGTH+1))
    {
      MOVE_GAIT_ORDER = STEP_GAIT_ORDER[gait_order];
      gait_order++;
      
      /*摆动腿和支撑腿足端轨迹在下一个1/3周期的变换，摆动腿增加2/3歩幅，支撑腿减少1/3歩幅*/
      for(int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
      {
	if( MOVE_GAIT_ORDER[leg_index]==1 )
	{
	  leg_step[leg_index] = leg_step[leg_index] + 2.0/3.0;
	}
	if ( MOVE_GAIT_ORDER[leg_index]==0 )
	{
	  leg_step[leg_index] = leg_step[leg_index] - 1.0/3.0;
	}
      }

    }
    
    if(gait_order == 3)
    {
      gait_order = 0;
    }
    
}
