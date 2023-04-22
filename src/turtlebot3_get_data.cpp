/*
  TESTING CODE BY JCTR WITH THE OBJECTIVE OT TEST MPC CONTROLLER IN A turtlebot3 ROBOT
*/

#include <turtlebot3_cpp/turtlebot3_get_data.hpp>

using std::placeholders::_1;

turtlebot3_MPC::turtlebot3_MPC() : Node("turtlebot3_mpc")
{
  subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&turtlebot3_MPC::topic_odom, this, _1));

  subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&turtlebot3_MPC::topic_imu, this, _1));

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

  timer_ = this->create_wall_timer(
    100ms, std::bind(&turtlebot3_MPC::trajectory, this));

  /*=========================================================================
                                  M MATRIX
  ==========================================================================*/
  this->M.block<3,3>(0,0) = Ad;
  this->M.block<3,1>(0,3) = Bd;

  /*=========================================================================
                                  N MATRIX
  ==========================================================================*/
  this->N.block<3,1>(0,0) = Bd;
  this->N(3,0) = 1;

  /*=========================================================================
                                  Q MATRIX
  ==========================================================================*/
  this->Q.block<1,3>(0,0) = Cd;

  /*=========================================================================
                                  F MATRIX
  ==========================================================================*/
  for(int i=0; i<30; i++)
  {
    MN2 = MN2 * M;
    F_1 = Q * MN2;
    for(int k=0; k<4; k++)
    {
      F(i,k) = F_1(0,k);
    }
  }

  /*=========================================================================
                                  H MATRIX
  ==========================================================================*/
  for(int i=0; i<25; i++)
  {
    for(int k=0; k<30-i; k++)
    {
      H(k+i,i) = F(k,3);
    }
  }

  /*=========================================================================
                                  ku MATRIX
  ==========================================================================*/
  ku1 = H.transpose() * Rw;
  ku2 = (ku1*H) + Qw;
  ku = ku2.inverse() * ku1;
}

void turtlebot3_MPC::topic_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // ODOM turtlebot3
  pos_x = msg->pose.pose.position.x;
  pos_y = msg->pose.pose.position.y;
}

void turtlebot3_MPC::topic_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
{
  // IMU turtlebot3
  x_linear_speed = imu->linear_acceleration.x;
  y_linear_speed = imu->linear_acceleration.y;
  z_linear_speed = imu->linear_acceleration.z;
  w_velocity = imu->angular_velocity.z;

  /*FOR THIS PROJECT WE USE IMU DATA BECAUSE ORIENTATION FROM ODOM IS WRONG*/
  quaternion_turtlebot3 =  Eigen::Quaternionf(imu->orientation.w,
                                        0,
                                        0,
                                        imu->orientation.z);

  eulerAngles_ = Quaternion2Euler(quaternion_turtlebot3);
  yaw = eulerAngles_(2);
}

void turtlebot3_MPC::trajectory()
{
  // WAY POINTS
  wpx_1 = goalx(ini);
  wpy_1 = goaly(ini);
  wpx_2 = goalx(ini+1);
  wpy_2 = goaly(ini+1);

  // DISTANCE VECTOR
  delta_wpx = wpx_2 - wpx_1;
  delta_wpy = wpy_2 - wpy_1;
  alpha = atan2(delta_wpy, delta_wpx);

  //========= MIDDLE POINT ========================================//
  PT_x = Ts*Vd*count*cos(alpha) + wpx_1;
  PT_y = Ts*Vd*count*sin(alpha) + wpy_1;

  // CONTROL ANGLE
  angle_to_goal = atan2(PT_y - pos_y, PT_x - pos_x);

  // DISTANCE
  delta_x1 = PT_x - pos_x;
  delta_y1 = PT_y - pos_y;
  delta_x2 = wpx_2 - pos_x;
  delta_y2 = wpy_2 - pos_y;

  absolute_x1 = pow(abs(delta_x1),2);
  absolute_y1 = pow(abs(delta_y1),2);
  absolute_x2 = pow(abs(delta_x2),2);
  absolute_y2 = pow(abs(delta_y2),2);

  distance = sqrt(absolute_x2 + absolute_y2);
  distance_PT = sqrt(absolute_x1 + absolute_y1);

  if(distance <= 0.5)
  {
    ini = ini + 1;
    count = 1;
  }

  if(distance_PT <= 0.5)
  {
    count = count + 1;
  }

  // SPEED CONTROL
  int k = 1;
  Vd = k*distance;

  if(Vd > 0.5)
    Vd = 0.5;

  if(Vd < 0.0)
    Vd = 0.0;

  //================== DISTANCE ERROR TO IMPLEMENT THE PD CONTROLLER ===========

  if(angle_to_goal > 0)
  {
    comp = angle_to_goal - 2*M_PI;
  }
  else
  {
    comp = angle_to_goal + 2*M_PI;
  }

	error1 = angle_to_goal - yaw;
	error2 = angle_to_goal + comp;

  if(abs(error1) > abs(error2))
  {
    error = error2;
  }
  else
  {
    error = error1;
  }

  // ========================== CONTROLLER =================================
  uk_pid = simple_PID(error);
  uk_mpc = simple_MPC(w_velocity, yaw, angle_to_goal);

  //printf("mpc: %f\n", uk_mpc);
  //printf("pid: %f\n", uk_pid);
  //printf("\n");

  // ==================== PUB MESSAGE =======================================

  // ACCION DE CONTROL
  auto msg = geometry_msgs::msg::Twist();

  if(uk_mpc >= 1){uk_mpc = 1;}
  if(uk_mpc <=-1){uk_mpc = -1;}

  if(ini+2 > goalx.size())
  {
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
  }
  else
  {
    msg.linear.x = 0.3;
    msg.angular.z = uk_mpc;
  }
  publisher_->publish(msg);
}
