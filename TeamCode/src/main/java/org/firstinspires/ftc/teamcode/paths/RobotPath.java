package org.firstinspires.ftc.teamcode.paths;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class RobotPath {
  public static class Waypoint {
    public final Point3d position;
    public final Vector3d obstacleNormal;
    public final double passLength;

    /* package private */ Waypoint(Point3d position, Vector3d obstacleNormal, double passLength) {
      this.position = position;
      this.obstacleNormal = obstacleNormal;
      this.passLength = passLength;
    }
  }
}
