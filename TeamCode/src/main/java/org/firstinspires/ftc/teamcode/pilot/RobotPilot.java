package org.firstinspires.ftc.teamcode.pilot;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import javax.vecmath.Matrix4d;

public interface RobotPilot {
  void setDestination(Matrix4d destination);
  void updateWithTags(Matrix4d cameraTransformRobotSpace, List<AprilTagDetection> detections);
  void updateWithOffset(Matrix4d offset);
  boolean tick();
  double getRobotBoundingRadius();
}
