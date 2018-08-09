#include "gait.h"
static const double PI=3.141592653;

Gait::Gait( void )
{
 // ros::param::get( "CYCLE_LENGTH", CYCLE_LENGTH );
  ros::param::get("MOVE_LENGTH", MOVE_LENGTH);
  ros::param::get("STEP_LENGTH", STEP_LENGTH);
  ros::param::get( "LEG_LIFT_HEIGHT", LEG_LIFT_HEIGHT );
  ros::param::get( "LEG_GAIT_ORDER", cycle_leg_number_ );
  ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
  ros::param::get("MOVE_GAIT_ORDER", MOVE_GAIT_ORDER);
  CYCLE_LENGTH = MOVE_LENGTH*2 + STEP_LENGTH*6;
  cycle_period_=0;
  leg_order_ = 0;
  period_seg = 0.3;

  current_time_=ros::Time::now(); 
  last_time_=ros::Time::now();
}

//每条摆动腿和支撑腿一个周期内的步幅控制
void Gait::cyclePeriod( const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet )
{
//   double period_distance, period_height;
//     period_distance = -cos( cycle_period_*PI/CYCLE_LENGTH );  //每条腿PI/CYCLE_LENGTH时间的步幅
//     period_height = 0.5 * sin( cycle_period_*PI/CYCLE_LENGTH ); //摆动腿PI/CYCLE_LENGTH时间抬起的高度
  
 /* //摆动腿和支撑腿歩幅控制
  for( int leg_index=0; leg_index<NUMBER_OF_LEGS; leg_index++ )
  {
    //摆动腿
    if( cycle_leg_number_[leg_index]==0 )
    {
      feet->foot[leg_index].position.x = base.x * period_distance;
      feet->foot[leg_index].position.y = base.y * period_distance;
      feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
      feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
    }
    //支撑腿
    if( cycle_leg_number_[leg_index]==1 )
    {
      feet->foot[leg_index].position.x = -base.x * period_distance;
      feet->foot[leg_index].position.y = -base.y * period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
    }
  }*/
 
 //重心移动
  if(cycle_period_<=MOVE_LENGTH)
  {
    double move_period_distance = 0.5 * cos( M_PI*cycle_period_/MOVE_LENGTH ) - 0.5;
    
    for(int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
    {
      feet->foot[leg_index].position.x = base.x * move_period_distance;
      feet->foot[leg_index].position.y = base.y * move_period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = base.theta * move_period_distance;
    }
  }
  
  //六条腿依次摆动
  if(cycle_period_ >MOVE_LENGTH && cycle_period_ <= (CYCLE_LENGTH-MOVE_LENGTH))
  {
    for(int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
    {
      if(MOVE_GAIT_ORDER[leg_index]==1)
      {
	int step_cout;
	if(leg_order_ == 0)
	{
	  step_cout = 5;
	}
	else
	{
	  step_cout = leg_order_-1;
	}
	int step_period = cycle_period_-MOVE_LENGTH-step_cout*STEP_LENGTH;
	ROS_INFO("step_cout: %d, step_period: %d", step_cout, step_period);
	double step_period_distance, step_period_height;
	if(step_period <= period_seg*STEP_LENGTH)
	{
	  step_period_distance = -1;
          step_period_height = - 0.5 * cos( M_PI * step_period / (period_seg * STEP_LENGTH)) + 0.5;
	}
	else if(step_period >= (1-period_seg)*STEP_LENGTH)
	{
	  step_period_distance = 1;
	  step_period_height = 0.5 * cos( M_PI * (step_period - (1-period_seg)*STEP_LENGTH) / (period_seg*STEP_LENGTH) ) + 0.5;
	}
	else
	{
	  step_period_distance = -cos( M_PI * (step_period - period_seg*STEP_LENGTH)/((1-2*period_seg)*STEP_LENGTH) );
	  step_period_height = 1;
	}
	feet->foot[leg_index].position.x = base.x * step_period_distance;
        feet->foot[leg_index].position.y = base.y * step_period_distance;
        feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * step_period_height;
        feet->foot[leg_index].orientation.yaw = base.theta * step_period_distance;
      }
    }
  }
  
  //重心前移
  if(cycle_period_ > (CYCLE_LENGTH-MOVE_LENGTH))
  {
    double move_period_distance = 0.5 * cos( M_PI*(cycle_period_-6*STEP_LENGTH-MOVE_LENGTH)/MOVE_LENGTH ) + 0.5;
    
    for(int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
    {
      feet->foot[leg_index].position.x = base.x * move_period_distance;
      feet->foot[leg_index].position.y = base.y * move_period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = base.theta * move_period_distance;
    }
  }
  
  

}
 
//摆动腿和支撑腿切换
void Gait::gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet )
{

    base.x = cmd_vel.linear.x ; 
    base.y = cmd_vel.linear.y ;
    base.theta = cmd_vel.angular.z ;


  
  // Low pass filter on the values to avoid jerky movements due to rapid value changes
    smooth_base_.x = base.x * 0.05 + ( smooth_base_.x * ( 1.0 - 0.05 ) );
    smooth_base_.y = base.y * 0.05 + ( smooth_base_.y * ( 1.0 - 0.05 ) );
    smooth_base_.theta = base.theta * 0.05 + ( smooth_base_.theta * ( 1.0 - 0.05 ) );
    if (base.x == 0 && base.y == 0 && base.theta == 0)
    {
      smooth_base_.x = 0;
      smooth_base_.y = 0;
      smooth_base_.theta = 0;
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
    
    //重心前移
    if( cycle_period_ == CYCLE_LENGTH )
    {
      cycle_period_ = 0;
      std::vector<int> move_gait_order(6, 0);
      MOVE_GAIT_ORDER = move_gait_order;
    } 
    
    //依次摆动五条腿
    if( cycle_period_ == ((MOVE_LENGTH+leg_order_*STEP_LENGTH)+1))
    {
      std::vector<int> move_gait_order(6, 0);
      move_gait_order[leg_order_] = 1;
      MOVE_GAIT_ORDER = move_gait_order;
      leg_order_++;
    }

    //重心前移动
    if(leg_order_ == 6)
    {
      leg_order_ = 0;
    }
    if( cycle_period_ == ((MOVE_LENGTH+6*STEP_LENGTH)+1))
    {
      std::vector<int> move_gait_order(6, 0);
      MOVE_GAIT_ORDER = move_gait_order;
    }
    
    std::vector<int>::iterator pd;
    std::cout<<cycle_period_<<" : ";
    for(pd = MOVE_GAIT_ORDER.begin(); pd != MOVE_GAIT_ORDER.end(); pd++)
    {
      std::cout<<*pd<<" ";
    }
    std::cout<<std::endl;
    
}



