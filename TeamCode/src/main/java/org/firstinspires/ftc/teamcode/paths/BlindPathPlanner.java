package org.firstinspires.ftc.teamcode.paths;

import java.util.ArrayDeque;
import java.util.Queue;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import org.firstinspires.ftc.teamcode.drive.MotorActionState;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;

public class BlindPathPlanner implements PathPlanner {
  private static final double UNRESTRICTED_PASS_LENGTH = 10000000.0;
  private DriveSystem driveSystem;
  private Queue<RobotWaypoint> waypointQueue;

  public BlindPathPlanner(DriveSystem driveSystem) {
    waypointQueue = new ArrayDeque();
    waypointQueue.add(new RobotWaypoint(new Point2d(6.0, 3.0), new Vector2d(0.0, -1.0),
      UNRESTRICTED_PASS_LENGTH));
    this.driveSystem = driveSystem;
  }

  public MotorActionState getNextAction() {
    RobotWaypoint waypoint = waypointQueue.poll();
    if (waypoint != null) {
      
    }
    return null;
  }
}
