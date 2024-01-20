package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;

import java.util.ArrayDeque;
import java.util.List;
import java.util.Queue;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

/**
 * Executes a queue of preset goals.
 * <p>
 * Does not respond to game state.
 */
public class BlindPathPlanner implements PathPlanner {
  private Queue<RobotGoal> goalQueue;
  private RobotNavigator navigator;
  private RobotLogger logger;
  private boolean stopped;
  public BlindPathPlanner(RobotLogger logger, RobotNavigator navigator) {
    this.logger = logger;
    goalQueue = new ArrayDeque<>();
    populateGoalQueue(goalQueue);
    this.navigator = navigator;
    stopped = false;
    processNextGoal();
  }

  private void processNextGoal() {
    logger.log("Processing next goal");
    RobotGoal goal = goalQueue.poll();
    if (goal == null) {
      stopped = true;
    } else {
      navigator.setGoal(goal);
    }
  }

  private static void populateGoalQueue(Queue<RobotGoal> goalQueue) {
    Matrix3d rot = new Matrix3d();

    rot.rotZ(0.0);
    goalQueue.add(new RobotGoal(new Matrix4d(rot, new Vector3d(0.0, 2.0, 0.0), 1.0)));

    rot.rotZ(Math.PI / 2);
    goalQueue.add(new RobotGoal(new Matrix4d(rot, new Vector3d(0.0, 2.0, 0.0), 1.0)));

    goalQueue.add(new RobotGoal(new Matrix4d(rot, new Vector3d(-4.0, 2.0, 0.0), 1.0)));

    goalQueue.add(new RobotGoal(new Matrix4d(rot, new Vector3d(0.0, 1.0, 0.0), 1.0)));

    rot.rotZ(0.0);
    goalQueue.add(new RobotGoal(new Matrix4d(rot, new Vector3d(0.0, 0.0, 0.0), 1.0)));
  }
  @Override
  public boolean tick() {
    if (stopped) {
      return true;
    }
    if (navigator.tick()) {
      processNextGoal();
    }
    return false;
  }

  @Override
  public boolean acceptSpikeTransforms(List<Matrix4d> transformList) {
    return true; // stop sending
  }
}
