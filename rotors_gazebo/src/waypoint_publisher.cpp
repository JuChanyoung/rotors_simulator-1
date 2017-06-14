/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <queue>

void construct_trajectory(std::queue<trajectory_msgs::MultiDOFJointTrajectory>& traj_queue, std::vector<Eigen::Vector3d>& desired_positions, std::vector<double>& desired_yaws)
{
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_temp;
	desired_positions.emplace_back(2,0,1);
	 desired_positions.emplace_back(2,2,1);
	  desired_positions.emplace_back(0, 2,1);
	  desired_positions.emplace_back(0,0,1);
	  desired_yaws.emplace_back(0);
	  desired_yaws.emplace_back(0);
	  desired_yaws.emplace_back(0);
	  desired_yaws.emplace_back(0);
	  for (int i = 0; i < 4; i++)
	  {
		  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_positions[i],
		          desired_yaws[i], &trajectory_msg_temp);
		  traj_queue.push(trajectory_msg_temp);
	  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh("");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  }
  else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  }
  else{
    ROS_ERROR("Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]\n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(2)),
                                   std::stof(args.at(3)));

  double desired_yaw = std::stof(args.at(4)) * DEG_2_RAD;
	//tbm:这个MultiDofJointTrajectory轨迹中之定义一个点
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);

  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());
  
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(5).sleep();

  std::queue<trajectory_msgs::MultiDOFJointTrajectory> traj_queue;
  std::vector<Eigen::Vector3d> desired_positions;
  std::vector<double> desired_yaws;
  construct_trajectory(traj_queue, desired_positions, desired_yaws);
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_temp;
  ros::Rate loop_rate(0.5);
  while(ros::ok() && !traj_queue.empty())
  {
	  trajectory_pub.publish(traj_queue.front());
	  trajectory_msg_temp = traj_queue.front();
	  traj_queue.pop();
	  traj_queue.push(trajectory_msg_temp);
	  loop_rate.sleep();
  }
	ros::spin();
  
  /*
  ros::Rate loop_rate(1);
  while(ros::ok())
  {
      trajectory_pub.publish(trajectory_msg);
      ros::spinOnce();
      loop_rate.sleep();
      ROS_INFO_THROTTLE(1, "I am publishing \n");
  }
  */
  return 0;
}
