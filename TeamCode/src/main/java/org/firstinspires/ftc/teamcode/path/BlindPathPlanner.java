package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MotorActionState;
import org.firstinspires.ftc.teamcode.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;

import java.util.ArrayDeque;
import java.util.Queue;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public class BlindPathPlanner implements PathPlanner {
  private static final double UNRESTRICTED_PASS_LENGTH = 10000000.0;
  private DriveSystem driveSystem;
  private Queue<RobotWaypoint> waypointQueue;
  private Queue<MotorActionState> actionBuffer;
  private RobotNavigator navigator;
  private RobotLogger logger;

  public BlindPathPlanner(DriveSystem driveSystem, RobotNavigator navigator, RobotLogger logger) {
    this.logger = logger;
    waypointQueue = new ArrayDeque<>();
    actionBuffer = new ArrayDeque<>();
    waypointQueue.add(new RobotWaypoint(new Point2d(6.0, 3.0), new Vector2d(0.0, -1.0),
            UNRESTRICTED_PASS_LENGTH));
    waypointQueue.add(new RobotWaypoint(new Point2d(8.0, 3.0), new Vector2d(0.0, -1.0),
            UNRESTRICTED_PASS_LENGTH));
    this.driveSystem = driveSystem;
    this.navigator = navigator;
  }

  public MotorActionState getNextAction() {
    MotorActionState bufferedAction = actionBuffer.poll();
    if (bufferedAction != null) {
      return bufferedAction;
    }
    RobotWaypoint waypoint = waypointQueue.poll();
    if (waypoint != null) {
      Point2d targetPoint = new Point2d();
      targetPoint.scaleAdd(driveSystem.getFootprintRadius(), waypoint.obstacleNormal,
        waypoint.position);
      logger.log("Pathing to: " + targetPoint.toString());
      Matrix3d targetRotation = new Matrix3d();
      targetRotation.setIdentity();
      Matrix4d targetTransform = navigator.convertToRobotSpace(new Matrix4d(targetRotation,
        new Vector3d(targetPoint.x, targetPoint.y, 0.0), 1.0));
      Vector3d targetTranslation = new Vector3d();
      logger.log("RS translation: " + targetTranslation);
      targetTransform.get(targetTranslation);
      return driveSystem.computeMove(new Vector2d(targetTranslation.x, targetTranslation.y), 1.0);
//      return driveSystem.computeMove(new Vector2d(targetPoint), 1.0); // wrong
//      return driveSystem.computeLinearSwivel(targetTransform, 1.0);
    }
    return null;
  }
}
