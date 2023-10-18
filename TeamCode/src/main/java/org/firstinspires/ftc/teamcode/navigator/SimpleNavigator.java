package org.firstinspires.ftc.teamcode.navigator;

import java.util.List;
import javax.vecmath.Point3d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.MotorActions;

public class SimpleNavigator {
  private Point3d position;
  private double rotation;

  @Override
  void updateFromTags(List<AprilTagDetection> detections) {

  }
  @Override
  void updateFromOffset(Point3d offset) {

  }
  @Override
  void adjustActions(MotorActions actions) {

  }
}
