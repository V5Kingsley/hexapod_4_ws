/*****************************************
 *     六足控制算法通用步态规划程序          *
 *     Copyright (c) V5_Lab, 2018        *
 *     Author:       Kingsley            *
 *     Version number:  0.00             *
 *     Date:                             *
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
  ros::param::get("THREE_GAIT_ORDER_1", THREE_GAIT_ORDER_1);
  ros::param::get("THREE_GAIT_ORDER_2", THREE_GAIT_ORDER_2);
  ros::param::get("FIVE_GAIT_ORDER_1", FIVE_GAIT_ORDER_1);
  ros::param::get("FIVE_GAIT_ORDER_2", FIVE_GAIT_ORDER_2);
  ros::param::get("FIVE_GAIT_ORDER_3", FIVE_GAIT_ORDER_3);
  ros::param::get("FIVE_GAIT_ORDER_4", FIVE_GAIT_ORDER_4);
  ros::param::get("FIVE_GAIT_ORDER_5", FIVE_GAIT_ORDER_5);
  ros::param::get("FIVE_GAIT_ORDER_6", FIVE_GAIT_ORDER_6);
  ros::param::get("LINEAR_X_MAX", linear_x_max);
  ros::param::get("LINEAR_Y_MAX", linear_y_max);
  ros::param::get("ANGULAR_Z_MAX", angular_z_max);
  smooth_base_.x = smooth_base_.y = smooth_base_.theta = 0;
  base.x = base.y = base.theta = 0;
 // ros::param::get("GAIT_NUM", GAIT_NUM);
  gn.param<int>("GAIT_NUM", GAIT_NUM, 3);
  STEP_GAIT_ORDER.resize(3);
  STEP_GAIT_ORDER[0] = STEP_GAIT_ORDER_1;
  STEP_GAIT_ORDER[1] = STEP_GAIT_ORDER_2;
  STEP_GAIT_ORDER[2] = STEP_GAIT_ORDER_3;
  ONE_STEP_LENGTH = ONE_STEP_LENGTH_ORIGIN;
  THREE_GAIT_ORDER.resize(2);
  THREE_GAIT_ORDER[0] = THREE_GAIT_ORDER_1;
  THREE_GAIT_ORDER[1] = THREE_GAIT_ORDER_2;
  FIVE_GAIT_ORDER.resize(6);
  FIVE_GAIT_ORDER[0] = FIVE_GAIT_ORDER_1;
  FIVE_GAIT_ORDER[1] = FIVE_GAIT_ORDER_2;
  FIVE_GAIT_ORDER[2] = FIVE_GAIT_ORDER_3;
  FIVE_GAIT_ORDER[3] = FIVE_GAIT_ORDER_4;
  FIVE_GAIT_ORDER[4] = FIVE_GAIT_ORDER_5;
  FIVE_GAIT_ORDER[5] = FIVE_GAIT_ORDER_6;
  cycle_period_=1;
  gait_order = 0;
  leg_step[6]={0};
  start_cycle = 1;
  stop_cycle = 0;
  stop_active = 0;
  lift_drop_length = 0.3;
  STEP_NUM = NUMBER_OF_LEGS/(NUMBER_OF_LEGS-GAIT_NUM);
  //swing_step = float(GAIT_NUM)/float(NUMBER_OF_LEGS);
  //support_step = 1- swing_step;
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
    step_cout = STEP_NUM-1;
  }
  else
  {
    step_cout = gait_order - 1;
  }
//   if (MOVE_GAIT_ORDER.empty())
//   {
//     ROS_FATAL("Please set the speed to 0 and restart.");
//     ros::shutdown();
//     return;
//   }
  
//   std::cout<<step_cout<<" : ";
//   std::vector<int>::iterator pd;
//   for(pd = MOVE_GAIT_ORDER.begin(); pd != MOVE_GAIT_ORDER.end(); pd++)
//   {
//     std::cout<<*pd<<" ";
//   }
//   std::cout<<std::endl;
//   bool step_change = std::abs(step_cout - step_cout_last_time);
//   step_cout_last_time = step_cout;

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
	period_distance = leg_step[leg_index] - swing_step;
	period_height = - 0.5 * cos( M_PI * step_period / LIFT_LENGTH) + 0.5;
	//ROS_INFO("lift, step_period: %d", step_period);
      }
      else if(step_period >= (ONE_STEP_LENGTH-DROP_LENGTH))
      {
	period_distance = leg_step[leg_index];
	period_height = 0.5 * cos( M_PI * (step_period - (LIFT_LENGTH+MOVE_LENGTH)) / DROP_LENGTH ) + 0.5;
	//ROS_INFO("drop, step_period: %d", step_period);
      }
      else
      {
	period_distance = - swing_step/2.0 * cos( M_PI * (step_period-LIFT_LENGTH) / MOVE_LENGTH ) + (leg_step[leg_index]-swing_step+swing_step/2.0);
	period_height = 1;
	//ROS_INFO("move, step_period: %d, period_distance: %f", step_period, period_distance);
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
	period_distance = leg_step[leg_index] + support_step;
      }
      else if(step_period >= (ONE_STEP_LENGTH-DROP_LENGTH))
      {
	period_distance = leg_step[leg_index];
      }
      else
      {
	period_distance = support_step/2.0 * cos(M_PI * (step_period-LIFT_LENGTH) / MOVE_LENGTH) + (support_step/2.0+leg_step[leg_index]);
      }  
      feet->foot[leg_index].position.x = base.x * period_distance;
      feet->foot[leg_index].position.y = base.y * period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
    }
  }
  
  //ROS_INFO("start_cycle: %d", start_cycle);
  
  //三角步态时，只需二分之一周期，即一个抬腿周期后即可停止；当处于步态更换停止周期时，当前不能属于开始周期。停止周期完成后，标识位复位，步态更新，每次停止复位后开始周期标志符都赋1
  if(GAIT_NUM==3)
  {
    if(stop_cycle == 1 && support_step==float(GAIT_NUM)/float(NUMBER_OF_LEGS) && (cycle_period_==ONE_STEP_LENGTH||cycle_period_==CYCLE_LENGTH) && start_cycle == 0)
    {
      stop_cycle = 0;
      cycle_period_=0;
      gait_order = 0;
      start_cycle = 1;
      gn.param<int>("GAIT_NUM", GAIT_NUM, 3);
      STEP_NUM = NUMBER_OF_LEGS/(NUMBER_OF_LEGS-GAIT_NUM);
      CYCLE_LENGTH = ONE_STEP_LENGTH * STEP_NUM;
    }
  }
  //不是三角步态时
  else
  {
    if(stop_cycle==1 && cycle_period_==CYCLE_LENGTH)
    {
      start_cycle = 1;
      stop_cycle = 0;
      gn.param<int>("GAIT_NUM", GAIT_NUM, 3);
      STEP_NUM = NUMBER_OF_LEGS/(NUMBER_OF_LEGS-GAIT_NUM);
      CYCLE_LENGTH = ONE_STEP_LENGTH * STEP_NUM;
      gait_order = 0;
      cycle_period_ = 0;
    }
  }
  
  ///开始周期完成，开始周期只针对三角步态
  if(start_cycle == 1 && cycle_period_== ONE_STEP_LENGTH)
  {
    start_cycle = 0;
  }
  
}
 
//摆动腿和支撑腿切换
void Gait::gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet )
{
/***获取步态*****/
  int gait_num; 
  gn.param<int>("GAIT_NUM", gait_num, 3);
  
  //当机器人处于静止状态时，直接更改步态
  if(smooth_base_.x == 0 && smooth_base_.y == 0 && smooth_base_.theta == 0) 
  {
    GAIT_NUM = gait_num;
  }
  //否则需要机器人走完停止周期时才更改步态，停止周期标志位赋1
  else  
  {
    if((gait_num-GAIT_NUM)!=0)
    {
      stop_cycle = 1;
    }
  }

  /********机器人速度改变时*******/
  if( (base.x - cmd_vel.linear.x) != 0 || (base.y - cmd_vel.linear.y) != 0 || (base.theta - cmd_vel.angular.z) != 0)
  { 
    //当发布的速度为0时，停止复位启动，仅进入一次
    if(cmd_vel.linear.x == 0 && cmd_vel.linear.y == 0 && cmd_vel.angular.z == 0 && stop_active == 0)
    {
      stop_cycle = 1;
      stop_active = 1;
    }
    
    //当不是处于停止复位周期时，将速度赋给机器人
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
  STEP_NUM = NUMBER_OF_LEGS/(NUMBER_OF_LEGS-GAIT_NUM);
  CYCLE_LENGTH = ONE_STEP_LENGTH * STEP_NUM;
  
//   if(GAIT_NUM == 3)
//   {
//     if(smooth_base_.x==0 && smooth_base_.y==0 && smooth_base_.theta==0)
//     {
//      for(int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
//      {
//        leg_step[leg_index] = 0;
//     }
//      gait_order = 0;
//      start_cycle = 1;
//     }
//   }
  
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
      /****************************************************/
      /*每个抬腿周期后更换摆动腿组和支撑腿组*/
    //  ROS_INFO("cycle_period_: %d, gait_order: %d", cycle_period_, gait_order);
    if(cycle_period_ == (gait_order*ONE_STEP_LENGTH+1))
    {   
      if(GAIT_NUM == 4)
      {
	MOVE_GAIT_ORDER = STEP_GAIT_ORDER[gait_order];
      }
      if(GAIT_NUM == 3)
      {
	MOVE_GAIT_ORDER = THREE_GAIT_ORDER[gait_order];
      }
      if(GAIT_NUM == 5)
      {
	MOVE_GAIT_ORDER = FIVE_GAIT_ORDER[gait_order];
      }
      gait_order++;
      
      /*摆动腿和支撑腿足端轨迹在下一个周期的变换，摆动腿增加歩幅，支撑腿减少歩幅*/
      /*摆动腿每次在变换周期时，歩幅增加float(GAIT_NUM)/float(NUMBER_OF_LEGS)，支撑腿歩幅减少1- swing_step*/
      /*三角步态时，开始周期和停止周期与其他步态一致，其他周期不一致，歩幅增加和减少的数值为1*/
      for(int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
      {
	if( MOVE_GAIT_ORDER[leg_index]==1 )
	{
	  if(GAIT_NUM==3 && stop_cycle==0 && start_cycle==0)  
	  {
	    leg_step[leg_index] = leg_step[leg_index] + 1;
	    swing_step = 1.0;
	    support_step = 1.0;
	  }
	  else
	  {
	    swing_step = float(GAIT_NUM)/float(NUMBER_OF_LEGS);
            support_step = 1- swing_step;
	    leg_step[leg_index] = leg_step[leg_index] + swing_step;
	  }
	}
	if ( MOVE_GAIT_ORDER[leg_index]==0 )
	{
	  if(GAIT_NUM==3 && stop_cycle==0 && start_cycle==0)
	  {
	    leg_step[leg_index] = leg_step[leg_index] - 1;
	    swing_step = 1.0;
	    support_step = 1.0;
	  }
	  else
	  {
	    swing_step = float(GAIT_NUM)/float(NUMBER_OF_LEGS);
            support_step = 1- swing_step;
	    leg_step[leg_index] = leg_step[leg_index] - support_step;
	  }
	}
//	ROS_INFO("leg_index: %d, leg_step: %f", leg_index, leg_step[leg_index]);
      }

    }
    
    if(gait_order == STEP_NUM)
    {
      gait_order = 0;
    }
      /********************************************************************/
      //给下一个period/CYCLE_LENGTH足端歩幅
      cyclePeriod( smooth_base_, feet);
      cycle_period_++;
    }
    
    //一个周期结束后从1计数
    if( cycle_period_==(CYCLE_LENGTH+1) )
    {
      cycle_period_=1;
    } 
    
    
}

