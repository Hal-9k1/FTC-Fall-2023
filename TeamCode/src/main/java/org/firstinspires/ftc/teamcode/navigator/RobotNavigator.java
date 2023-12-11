package org.firstinspires.ftc.teamcode.navigator;

import org.firstinspires.ftc.teamcode.path.RobotGoal;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;

/**
 * Given a {@link RobotGoal} and information about obstacles on the field, plots a path and supplies
 * a sequence of {@link RobotWaypoint}s to a {@link RobotPilot}.
 */
public interface RobotNavigator {
  /**
   * Sets a goal to navigate to.
   * @param goal The goal to navigate to.
   */
  void setGoal(RobotGoal goal);

  /**
   * Updates the navigator.
   * <p>
   * Navigates towards the current goal, taking recently discovered obstacles into consideration.
   * Delegates the calculation of bearings and movement to a RobotPilot.
   * @return Whether the navigator has reached its goal.
   */
  boolean tick();
  // TODO: notify about obstacles
}
