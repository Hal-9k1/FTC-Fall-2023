package org.firstinspires.ftc.teamcode.path;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/* pkg private */ class RobotWaypoint {
  public final Point2d position;
  public final Vector2d obstacleNormal;
  public final double passLength;

  /* pkg private */ RobotWaypoint(Point2d position, Vector2d obstacleNormal, double passLength) {
    this.position = position;
    this.obstacleNormal = obstacleNormal;
    this.passLength = passLength;
  }
}
