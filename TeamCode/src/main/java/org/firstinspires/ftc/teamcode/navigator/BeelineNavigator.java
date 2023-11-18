package org.firstinspires.ftc.teamcode.navigator;

import org.firstinspires.ftc.teamcode.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.path.RobotGoal;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;

import java.util.ArrayDeque;
import java.util.Queue;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public class BeelineNavigator implements RobotNavigator {
  // don't worry about it, we'll fit:
  private static final double UNRESTRICTED_PASS_LENGTH = 10000000000.0;
  private RobotGoal goal;
  private Queue<RobotWaypoint> waypoints;
  private RobotPilot pilot;
  private RobotLogger logger;
  public BeelineNavigator(RobotLogger logger, RobotPilot pilot) {
    this.logger = logger;
    goal = null;
    waypoints = new ArrayDeque<>();
    this.pilot = pilot;
  }
  @Override
  public void setGoal(RobotGoal goal) {
    this.goal = goal;
    waypoints.clear();
    Vector3d translation = new Vector3d();
    goal.getTransform().get(translation);
    waypoints.add(new RobotWaypoint(new Point2d(translation.x, translation.y),
      new Vector2d(0.0, 0.0),
      UNRESTRICTED_PASS_LENGTH));
    progressWaypoint(getNextWaypoint());
  }

  private void progressWaypoint(RobotWaypoint waypoint) {
    pilot.setDestination(getDestination(waypoint));
    // TODO: do arm stuff with waypoint
  }

  private RobotWaypoint getNextWaypoint() {
    logger.log("Getting next waypoint");
    if (goal == null) {
      throw new IllegalStateException("Goal has not been set.");
    }
    return waypoints.poll();
  }
  private Matrix4d getDestination(RobotWaypoint waypoint) {
    Vector2d translation = new Vector2d(waypoint.obstacleNormal);
    translation.scaleAdd(pilot.getRobotBoundingRadius(), waypoint.position);
    Matrix3d rotation = new Matrix3d();
    rotation.setIdentity();
    return new Matrix4d(rotation, new Vector3d(translation.x, translation.y, 0.0), 1.0);
  }
  @Override
  public boolean tick() {
    if (pilot.tick()) {
      RobotWaypoint waypoint = getNextWaypoint();
      if (waypoint == null) {
        return true;
      } else {
        progressWaypoint(waypoint);
      }
    }
    return false;
  }
}
