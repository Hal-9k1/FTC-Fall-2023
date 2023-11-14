package org.firstinspires.ftc.teamcode.navigator;

import java.util.List;
import javax.vecmath.Matrix4d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.MotorActionState;

public interface RobotNavigator {
  void updateWithTags(Matrix4d cameraTransform, List<AprilTagDetection> detections);
  void updateWithOffset(Matrix4d offset);
  void adjustActions(MotorActionState actions);
  Matrix4d convertToRobotSpace(Matrix4d worldSpaceTransform);
}
