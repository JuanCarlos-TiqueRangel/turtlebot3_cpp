// PROJECT DEVELOPED BY JUAN CARLOS TIQUE RANGEL

#ifndef TURTLEBOT3_CPP__TURTLEBOT3_GET_DATA_HPP_
#define TURTLEBOT3_CPP__TURTLEBOT3_GET_DATA_HPP_

#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <cmath> // LIBRARY TO USE sqrt, atan2
#include <cstdlib> // LIBRARY TO USE ABS FUNCTION

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class turtlebot3_MPC : public rclcpp::Node
{
  public:
    turtlebot3_MPC();

    double simple_MPC(double w, double yaw, double ref)
    {
      Eigen::Matrix<double, 30, 1> _ref = Eigen::Matrix<double, 30, 1>::Ones() * ref;

      if((ref - yaw) > M_PI)
      {
        ref = ref - 2*M_PI;
      }
      if((ref - yaw) < -M_PI)
      {
        ref = ref + 2*M_PI;
      }

      zk.block<3,1>(0,0) = xk;
      zk(3,0) = uk_1;

      du1 = _ref - (F * zk);
      deltaU = ku(0,Eigen::seq(0,Eigen::placeholders::last)) * du1;

      // CONTROLL ACCION UK
      uk = uk_1 + deltaU;
      uk_1 = uk;

      // STATE SPACE VARAIBLES
      delta_w = w - w_velocity_1;
      acc = delta_w/Ts;

      xk(0,0) = acc;
      xk(1,0) = w;
      xk(2,0) = yaw;

      w_velocity_1 = w;
      return uk;
    }

    double simple_PID(double error)
    {
      Up = kp*error;
    	Ui = ki*error + Ui_1*Ts;
    	Ud = (kd/Ts)*(error-error_1);
    	U = Up+Ui+Ud;

      error_1 = error;
      Ui_1 = Ui;

      return U;
    }

    Eigen::Vector3f Quaternion2Euler(Eigen::Quaternion<float> q)
    {
      auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      return euler;
    }

  private:
    // ODOM DATA
    double pos_x, pos_y;

    // IMU DATA
    double x_linear_speed;
    double y_linear_speed;
    double z_linear_speed;
    double w_velocity, w_velocity_1;
    double yaw, yaw_1;

    // TRAYECTORY DATA
    double distance;
    double distance_PT;
    int ini=0;
    double wpx_1, wpy_1, wpx_2, wpy_2;
    int count;

    double delta_wpx, delta_wpy;
    double alpha;
    double PT_x, PT_y;

    double Ts=0.1, Vd;

    double angle_to_goal;

    double delta_x1, delta_y1, delta_x2, delta_y2;
    double absolute_x1, absolute_y1, absolute_x2, absolute_y2;

    // CONTROLLER WEIGHTS
    int Np = 25;
    int Nc = 24;

    // CONTROLLER VARIABLES
    double error, error_1, error1, error2;
    double comp, uk, uk_1, uk_mpc;
    double deltaU2, deltaU;

    // STATE SAPCE VARIABLES
    double delta_pos, delta_w, acc, acc_1, delta_acc;

    // PID VARIABLES
    double kp = 1.1;
    double ki = 0.0;
    double kd = 0.5;
    double uk_pid, Up, Ui, Ud, U, Ui_1;

    Eigen::Matrix<double, 3, 3> Ad = (Eigen::Matrix<double, 3, 3>() <<
                                    0.7615, -0.1546, 0,
                                    0.1757, 0.9838, 0,
                                    0.01836, 0.1989, 1).finished();

    Eigen::Matrix<double, 3, 1> Bd = (Eigen::Matrix<double, 3, 1>() <<
                                    0.1757, 0.01836, 0.001251).finished();

    Eigen::Matrix<double, 1, 3> Cd = (Eigen::Matrix<double, 1, 3>() <<
                              0, 0, 0.1292).finished();

    Eigen::Matrix<double, 3, 1> xk = Eigen::Matrix<double, 3, 1>();
    Eigen::Matrix<double, 3, 1> xk_1 = Eigen::Matrix<double, 3, 1>();
    Eigen::Matrix<double, 4, 1> zk = Eigen::Matrix<double, 4, 1>();

    // EXTENDED MODEL
    Eigen::Matrix<double, 4, 4> M = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 4, 1> N = Eigen::Matrix<double, 4, 1>();
    Eigen::Matrix<double, 1, 4> Q = Eigen::Matrix<double, 1, 4>::Identity();
    Eigen::Matrix<double, 30, 30> Rw = Eigen::Matrix<double, 30, 30>::Identity() *  10.1;
    Eigen::Matrix<double, 25, 25> Qw = Eigen::Matrix<double, 25, 25>::Identity() * 10e-6;
    Eigen::Matrix<double, 30, 4> F = Eigen::Matrix<double, 30, 4>();
    Eigen::Matrix<double, 30, 25> H = Eigen::Matrix<double, 30, 25>();
    Eigen::Matrix<double, 4, 4> MN2 = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 1, 4> F_1 = Eigen::Matrix<double, 1, 4>::Zero();

    // CONTROLLER MATRIX
    Eigen::Matrix<double, 25, 30> ku1 = Eigen::Matrix<double, 25, 30>();
    Eigen::Matrix<double, 25, 25> ku2 = Eigen::Matrix<double, 25, 25>();
    Eigen::Matrix<double, 25, 30> ku = Eigen::Matrix<double, 25, 30>();
    Eigen::Matrix<double, 30, 1> du1 = Eigen::Matrix<double, 30, 1>();


    // TRAJECTORY GENERATOR
    Eigen::Matrix<int, 1, 7> goalx = (Eigen::Matrix<int, 1, 7>() <<
                              0,5,5,0,0,5,0).finished();

    Eigen::Matrix<int, 1, 7> goaly = (Eigen::Matrix<int, 1, 7>() <<
                              0,0,5,5,10,10,0).finished();

    Eigen::Matrix<double, 30, 25> H_1 = Eigen::Matrix<double, 30, 25>();

    Eigen::Quaternionf quaternion_turtlebot3;
    Eigen::Vector3f eulerAngles_;

    Eigen::MatrixXd result;
    Eigen::MatrixXd Y_;


    void topic_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void topic_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu);

    //publisher node
    void trajectory();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr subscription_imu;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

#endif  // turtlebot3_GET_DATA_HPP_
