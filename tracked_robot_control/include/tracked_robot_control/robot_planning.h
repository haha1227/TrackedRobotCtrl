#ifndef ROBOT_PLANNING_H
#define ROBOT_PLANNING_H
#include <trajectory_msgs/JointTrajectory.h>


class robot_planning
{
public:
  robot_planning();
  void planPTP(trajectory_msgs::JointTrajectory& joint_trajectory,
               double velocity_scaling_factor = 1.0,
               const double acceleration_scaling_factor = 1,
               double sampling_time = 0.04);
  const double MIN_MOVEMENT = 0.09;
};

#endif // ROBOT_PLANNING_H
