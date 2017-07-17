#ifndef CARROT_PLANNER_H_
#define CARROT_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace carrot_planner{

  class CarrotPlanner : public nav_core::BaseGlobalPlanner {
    public:
      CarrotPlanner();
      CarrotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      bool makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      
	  ros::NodeHandle n;
	  ros::Subscriber sub = n.subscribe("/odom", 1000, &CarrotPlanner::velSub, this);
	  void velSub(const nav_msgs::Odometry::ConstPtr& msg);
	  
	  nav_msgs::Odometry lastOdom;
	  float goalx, goaly;
	  bool odomSet, inner;
	  float alpha, beta, gamma;
	  	double initx, inity, initxs;
	      
    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      double step_size_, min_dist_from_robot_;
      costmap_2d::Costmap2D* costmap_;
      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use

      double footprintCost(double x_i, double y_i, double theta_i);

      bool initialized_;
  };
};  
#endif
