package org.firstinspires.ftc.teamcode.navigator;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import org.firstinspires.ftc.teamcode.drive.MotorActionState;

public class SimpleNavigator implements RobotNavigator {
  private Matrix4d robotTransform;
  private Map<Integer, Matrix4d> tagTransformsWorldSpace;

  public SimpleNavigator(Matrix4d initialRobotTransform, AprilTagLibrary tagLibrary) {
    robotTransform = new Matrix4d(initialRobotTransform);
    tagTransformsWorldSpace = new HashMap<Integer, Matrix4d>();
    for (AprilTagMetadata tagData : tagLibrary.getAllTags()) {
      Vector3d tagPosition = new Vector3d(
        DistanceUnit.METER.fromUnit(tagData.distanceUnit, tagData.fieldPosition.get(0)),
        DistanceUnit.METER.fromUnit(tagData.distanceUnit, tagData.fieldPosition.get(1)),
        DistanceUnit.METER.fromUnit(tagData.distanceUnit, tagData.fieldPosition.get(2))
      );
      MatrixF ftcTagRot = tagData.fieldOrientation.toMatrix();
      Matrix3d tagRotation = new Matrix3d(
        ftcTagRot.get(0, 0), ftcTagRot.get(0, 1), ftcTagRot.get(0, 2),
        ftcTagRot.get(1, 0), ftcTagRot.get(1, 1), ftcTagRot.get(1, 2),
        ftcTagRot.get(2, 0), ftcTagRot.get(2, 1), ftcTagRot.get(2, 2)
      );
      tagTransformsWorldSpace.put(tagData.id, new Matrix4d(tagRotation, tagPosition, 1.0));
    }
  }

  @Override
  void updateFromTags(Matrix4d cameraTransformRobotSpace, List<AprilTagDetection> detections) {
    // WS = world space, RS = robot space, CS = camera space
    double sumRotation = 0; // radians
    Point2d sumPosition = new Point2d();
    for (AprilTagDetection detection : detections) {
      // from ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_advanced_use/apriltag-advanced-use.html
      Matrix3d tagRotCS = new Matrix3d().setIdentity();
      tagRotCS // assuming rotation order XYZ
        .mul(new Matrix3d().rotZ(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES,
          detection.ftcPose.yaw))
        .mul(new Matrix3d().rotY(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES,
          detection.ftcPose.roll))
        .mul(new Matrix3d().rotX(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES,
          detection.ftcPose.pitch));
      Vector3d tagPosCS = new Vector3d(detection.ftcPose.x, detection.ftcPose.y,
        detection.ftcPose.z);
      // tagWS = robotWS * cameraRS * tagCS
      // robotWS = tagWS * tagCS^-1 * cameraRS^-1
      Matrix4d tagCS = new Matrix4d(tagRotCS, tagPosCS, 1.0);
      Matrix4d tagCSInv = new Matrix4d();
      tagCSInv.invert(tagCS);
      Matrix4d cameraRSInv = new Matrix4d();
      cameraRSInv.invert(cameraTransformRobotSpace);
      Matrix4d tagWS = tagTransformsWorldSpace.get(detection.id);
      Matrix4d robotWS = new Matrix4d(tagWS);
      robotWS.mul(tagCSInv);
      robotWS.mul(cameraRSINV);

      Vector3d robotPosWS = new Vector3d();
      robotPosWS.get(robotWS);
      // TODO: what to do if these calculations tell us we're below the floor or floating?
      Point2d robotPosFlatWS = new Point2d(robotPosWS.x, robotPos.y);
      sumPosition.add(robotPosFlatWS);
      double robotYawWS = ; // radians
      sumRotation += robotYawWS;
    }
    double avgRotation = sumRotation / detections.size();
    Point2d avgPosition = sumPosition.scale(1.0 / detections.size()); 
    Matrix3d avgRotationMat = new Matrix3d();
    avgRotationMat.rotZ(avgRotation);
    // TODO: old transform should be incorporated somehow instead of just being overwritten
    robotTransform = new Matrix4d(avgRotationMat, new Vector3d(avgPosition.x, avgPosition.y, 0.0));
  }
  @Override
  void updateFromOffset(Matrix4d offset) {
    robotTransform.multiply(offset);
  }
  @Override
  void adjustActions(MotorActionState actions) {
    
  }
  @Override
  Matrix4d convertToRobotSpace(Matrix4d worldSpaceTransform) {
    // worldSpaceTransform = robotTransform * robotSpaceTransform
    // robotTransform^-1 * worldSpaceTransform = robotSpaceTransform
    Matrix4d result = new Matrix4d();
    result.invert(robotTransform);
    result.mul(worldSpaceTransform);
    return result;
  }
}
