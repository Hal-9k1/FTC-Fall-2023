package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;

import java.util.List;

import javax.vecmath.Matrix4d;

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

  /**
   * Informs the planner of spikes that the robot sees.
   * <p>
   * Gives the planner a list of transforms of any detected spikes. The planner may decide it no
   * longer requires information about spikes, and the caller may or may not subsequently continue
   * calling this method.
   * @param transformList A list of transforms of any detected spikes.
   * @return Whether the robot may stop looking for spikes.
   */
  boolean acceptSpikeTransforms(List<Matrix4d> transformList);
}
