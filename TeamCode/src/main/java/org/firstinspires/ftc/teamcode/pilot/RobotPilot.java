package org.firstinspires.ftc.teamcode.pilot;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import javax.vecmath.Matrix4d;

public interface RobotPilot {
  void setDestination(Matrix4d destination);
  void updateWithTags(Matrix4d cameraTransformRobotSpace, List<AprilTagDetection> detections);
  void updateWithOffset(Matrix4d offset);
  void updateWithIMU(IMU imu);
  boolean tick();
  double getRobotBoundingRadius();
  default void addTelemetry(Telemetry telemetry) {}
}
