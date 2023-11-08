package org.firstinspires.ftc.teamcode.navigator;

import org.firstinspires.ftc.teamcode.path.RobotGoal;

public interface RobotNavigator {
  void setGoal(RobotGoal goal);
  RobotWaypoint getNextWaypoint();
  // TODO: notify about obstacles
}
