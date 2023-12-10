package org.firstinspires.ftc.teamcode.navigator;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/* pkg private */ class RobotWaypoint {
  public final Point2d position;
  public final Vector2d obstacleNormal;
  public final double passLength;
  public double yaw;

  /* pkg private */ RobotWaypoint(Point2d position, Vector2d obstacleNormal, double passLength, double yaw) {
    this.position = position;
    this.obstacleNormal = obstacleNormal;
    this.passLength = passLength;
    this.yaw = yaw;
  }
}
