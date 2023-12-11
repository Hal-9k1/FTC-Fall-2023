package org.firstinspires.ftc.teamcode.pilot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MatrixMagic;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.logging.RobotLogger;
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

/**
 * Localizes the robot using known AprilTags and offset transformations and nudges the robot to be
 * within epsilons of the current goal.
 */
public class SimplePilot implements RobotPilot {
  private static final double DISTANCE_EPSILON = 0.1;
  private static final double ANGLE_EPSILON = 0.1;
  private final DriveSystem driveSystem;
  private Matrix4d robotTransformFS;
  private Matrix4d robotTransformAS;
  private Matrix4d destinationFS;
  private Map<Integer, Matrix4d> tagTransformsWS;
  private RobotLogger logger;

  public SimplePilot(RobotLogger logger, DriveSystem driveSystem, Matrix4d allianceOriginTransform,
                     Matrix4d initialRobotTransform, AprilTagLibrary tagLibrary) {
    this.logger = logger;
    this.driveSystem = driveSystem;
    robotTransformFS = new Matrix4d(initialRobotTransform);
    robotTransformAS = new Matrix4d();
    destinationFS = new Matrix4d();
    tagTransformsWS = new HashMap<Integer, Matrix4d>();
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
      Matrix4d tagTransform = new Matrix4d(tagRotation, tagPosition, 1.0); // alliance space
      tagTransform.mul(allianceOriginTransform); // field space
      tagTransformsWS.put(tagData.id, tagTransform);
    }
  }

  /**
   * Sets a destination the pilot will attempt to move directly to.
   * @param destination The field space transform to move to.
   */
  @Override
  public void setDestination(Matrix4d destination) {
    destinationFS.set(destination);
    robotTransformAS.setIdentity();
    driveSystem.startNewAction();
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
    Matrix4d newRobotTransformAS = driveSystem.getActionSpaceTransform();
    updateWithOffset(MatrixMagic.invMul(robotTransformAS, newRobotTransformAS));
    robotTransformAS = newRobotTransformAS;

    Matrix4d destinationRS = convertToRS(destinationFS);
    Vector3d destTranslationRS = new Vector3d();
    destinationRS.get(destTranslationRS);
    double destAngleRS = MatrixMagic.getYaw(destinationRS);
    if (destTranslationRS.lengthSquared() < DISTANCE_EPSILON && destAngleRS < ANGLE_EPSILON) {
      logger.log("Reached destination");
      driveSystem.halt();
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
    return MatrixMagic.invMul(robotTransformFS, transformFS);
  }
  @Override
  public double getRobotBoundingRadius() {
    return driveSystem.getRobotBoundingRadius();
  }

  @Override
  public void addTelemetry(Telemetry telemetry) {
    Vector3d robotTranslationFS = new Vector3d();
    robotTransformFS.get(robotTranslationFS);
    double robotRotationFS = MatrixMagic.getYaw(robotTransformFS);
    telemetry.addData("Robot position (meters)", robotTranslationFS);
    telemetry.addData("Robot rotation (radians)", robotRotationFS);

    Vector3d destinationTranslationFS = new Vector3d();
    destinationFS.get(destinationTranslationFS);
    double destinationRotationFS = MatrixMagic.getYaw(destinationFS);
    telemetry.addData("Destination translation (field space, meters)", destinationTranslationFS);
    telemetry.addData("Destination rotation (field space, radians)", destinationRotationFS);

    Vector3d robotTranslationAS = new Vector3d();
    driveSystem.getActionSpaceTransform().get(robotTranslationAS);
    double robotRotationAS = MatrixMagic.getYaw(driveSystem.getActionSpaceTransform());
    telemetry.addData("Robot translation (action space, meters)", robotTranslationAS);
    telemetry.addData("Robot rotation (action space, radians)", robotRotationAS);
  }
}
