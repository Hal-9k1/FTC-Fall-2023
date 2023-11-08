package org.firstinspires.ftc.teamcode.pilot;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MatrixMagic;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

public class SimplePilot implements RobotPilot {
  private static final double DISTANCE_EPSILON = 0.1;
  private static final double ANGLE_EPSILON = 0.1;
  private final DriveSystem driveSystem;
  private Matrix4d robotTransformFS;
  private Matrix4d robotTransformAS;
  private Matrix4d destinationFS;
  private Map<Integer, Matrix4d> tagTransformsWS;
  private boolean stopped;

  public SimplePilot(DriveSystem driveSystem, Matrix4d initialRobotTransform, AprilTagLibrary tagLibrary) {
    this.driveSystem = driveSystem;
    robotTransformFS = new Matrix4d(initialRobotTransform);
    destinationFS = new Matrix4d();
    tagTransformsWS = new HashMap<Integer, Matrix4d>();
    stopped = true;
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
      tagTransformsWS.put(tagData.id, new Matrix4d(tagRotation, tagPosition, 1.0));
    }
  }

  /**
   * Sets a destination the pilot will attempt to move directly to.
   * @param destination The field space transform to move to.
   */
  @Override
  public void setDestination(Matrix4d destination) {
    destinationFS.set(destination);
    stopped = false;
  }

  @Override
  public void updateWithTags(Matrix4d cameraTransformRobotSpace, List<AprilTagDetection> detections) {
    // WS = world space, RS = robot space, CS = camera space
    double sumRotation = 0; // radians
    Point2d sumPosition = new Point2d();
    int knownDetectedTags = 0;
    for (AprilTagDetection detection : detections) {
      // from https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_advanced_use/apriltag-advanced-use.html
      Matrix3d tagRotCS = new Matrix3d();
      tagRotCS.setIdentity();
      Matrix3d rotZMat = new Matrix3d();
      rotZMat.rotZ(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES,
        detection.ftcPose.yaw));
      Matrix3d rotYMat = new Matrix3d();
      rotYMat.rotY(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES,
          detection.ftcPose.roll));
      Matrix3d rotXMat = new Matrix3d();
      rotXMat.rotX(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES,
          detection.ftcPose.pitch));
      // assuming rotation order XYZ
      tagRotCS.mul(rotZMat);
      tagRotCS.mul(rotYMat);
      tagRotCS.mul(rotXMat);
      Vector3d tagPosCS = new Vector3d(detection.ftcPose.x, detection.ftcPose.y,
        detection.ftcPose.z);
      // tagWS = robotWS * cameraRS * tagCS
      // robotWS = tagWS * tagCS^-1 * cameraRS^-1
      Matrix4d tagCS = new Matrix4d(tagRotCS, tagPosCS, 1.0);
      Matrix4d tagCSInv = new Matrix4d();
      tagCSInv.invert(tagCS);
      Matrix4d cameraRSInv = new Matrix4d();
      cameraRSInv.invert(cameraTransformRobotSpace);
      Matrix4d tagWS = tagTransformsWS.get(detection.id);
      Matrix4d robotWS = new Matrix4d(tagWS);
      robotWS.mul(tagCSInv);
      robotWS.mul(cameraRSInv);

      Vector3d robotPosWS = new Vector3d();
      robotWS.get(robotPosWS);
      // TODO: what to do if these calculations tell us we're below the floor or floating?
      Point2d robotPosFlatWS = new Point2d(robotPosWS.x, robotPosWS.y);
      sumPosition.add(robotPosFlatWS);
      double robotYawWS = MatrixMagic.getYaw(robotWS);
      sumRotation += robotYawWS;
      knownDetectedTags++;
    }
    double avgRotation = sumRotation / knownDetectedTags;
    Point2d avgPosition = new Point2d();
    avgPosition.scale(1.0 / knownDetectedTags, sumPosition);
    Matrix3d avgRotationMat = new Matrix3d();
    avgRotationMat.rotZ(avgRotation);
    // TODO: old transform should be incorporated somehow instead of just being overwritten
    robotTransformFS = new Matrix4d(avgRotationMat,
      new Vector3d(avgPosition.x, avgPosition.y, 0.0), 1.0);
  }
  @Override
  public void updateWithOffset(Matrix4d offset) {
    robotTransformFS.mul(offset);
  }
  @Override
  public boolean tick() {
    if (stopped) {
      return false;
    }
    Matrix4d newRobotTransformAS = driveSystem.getActionSpaceTransform();
    Matrix4d deltaASTransform = new Matrix4d();
    deltaASTransform.invert(robotTransformAS);
    deltaASTransform.mul(newRobotTransformAS);
    updateWithOffset(deltaASTransform);

    Matrix4d destinationRS = convertToRS(destinationFS);
    Vector3d destTranslationRS = new Vector3d();
    destinationRS.get(destTranslationRS);
    double destAngleRS = MatrixMagic.getYaw(destinationRS);
    if (destTranslationRS.lengthSquared() < DISTANCE_EPSILON || destAngleRS < ANGLE_EPSILON) {
      driveSystem.halt();
      stopped = true;
      return true;
    } else {
      driveSystem.swivel(destinationRS, 1.0);
      driveSystem.exec();
      return false;
    }
  }

  private Matrix4d convertToRS(Matrix4d transformFS) {
    // transformFS = robotTransform * transformRS
    // robotTransform^-1 * transformFS = transformRS
    Matrix4d result = new Matrix4d();
    result.invert(robotTransformFS);
    result.mul(transformFS);
    return result;
  }
}
