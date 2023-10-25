package org.firstinspires.ftc.teamcode.path;

import java.util.ArrayDeque;
import java.util.Queue;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.firstinspires.ftc.teamcode.drive.MotorActionState;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;

public class BlindPathPlanner implements PathPlanner {
  private static final double UNRESTRICTED_PASS_LENGTH = 10000000.0;
  private DriveSystem driveSystem;
  private Queue<RobotWaypoint> waypointQueue;
  private RobotNavigator navigator;

  public BlindPathPlanner(DriveSystem driveSystem, RobotNavigator navigator) {
    waypointQueue = new ArrayDeque<>();
    waypointQueue.add(new RobotWaypoint(new Point2d(6.0, 3.0), new Vector2d(0.0, -1.0),
      UNRESTRICTED_PASS_LENGTH));
    this.driveSystem = driveSystem;
    this.navigator = navigator;
  }

  public MotorActionState getNextAction() {
    RobotWaypoint waypoint = waypointQueue.poll();
    if (waypoint != null) {
      Point2d targetPoint = new Point2d();
      targetPoint.scaleAdd(driveSystem.getFootprintRadius(), waypoint.obstacleNormal,
        waypoint.position);
      Matrix3d targetRotation = new Matrix3d();
      targetRotation.setIdentity();
      Matrix4d targetTransform = navigator.convertToRobotSpace(new Matrix4d(targetRotation,
        new Vector3d(targetPoint.x, targetPoint.y, 0.0), 1.0));
      return driveSystem.computeLinearSwivel(targetTransform, 1.0);
    }
    return null;
  }
}
