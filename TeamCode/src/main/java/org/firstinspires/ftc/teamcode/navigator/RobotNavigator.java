package org.firstinspires.ftc.teamcode.navigator;

import java.util.List;
import javax.vecmath.Matrix4d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.MotorActions;

public interface RobotNavigator {
  void updateFromTags(Matrix4d cameraTransform, List<AprilTagDetection> detections);
  void updateFromOffset(Matrix4d offset);
  void adjustActions(MotorActions actions);
  Matrix4d convertToRobotSpace(Matrix4d worldSpaceTransform);
}
