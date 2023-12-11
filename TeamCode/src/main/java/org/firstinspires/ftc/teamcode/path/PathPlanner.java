package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;

/**
 * Given game state, produces a sequence of {@link RobotGoal}s to tell the robot where to go and
 * what to do.
 */
public interface PathPlanner {
  /**
   * Updates the planner.
   * <p>
   * Produces a goal, taking recently discovered game state into consideration.
   * Then instructs a {@link RobotNavigator} to execute the goal.
   * @return Whether the planner is out of goals.
   */
  boolean tick();
  // TODO: add interface to tell the planner about field state
}
