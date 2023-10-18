package org.firstinspires.ftc.teamcode.navigator;

import java.util.List;
import javax.vecmath.Point3d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.MotorActions;

public interface RobotNavigator {
  void updateFromTags(List<AprilTagDetection> detections);
  void updateFromOffset(Point3d offset);
  void adjustActions(MotorActions actions);
}
