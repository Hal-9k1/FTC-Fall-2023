package org.firstinspires.ftc.teamcode.paths;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

public class RobotWaypoint {
  public final Point2d position;
  public final Vector2d obstacleNormal;
  public final double passLength;

  /* pkg private */ Waypoint(Point2d position, Point2d obstacleNormal, double passLength) {
    this.position = position;
    this.obstacleNormal = obstacleNormal;
    this.passLength = passLength;
  }
}
