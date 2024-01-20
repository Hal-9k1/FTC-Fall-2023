package org.firstinspires.ftc.teamcode.pilot;

import com.qualcomm.robotcore.hardware.IMU;

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

import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

/**
 * Implements a pilot that moves toward a destination angle and position until it is within an
 * epsilon of each parameter.
 */
public abstract class
BasePilot implements RobotPilot {
    private static final double DISTANCE_EPSILON = 0.01; // meters
    private static final double ANGLE_EPSILON = 0.01; // radians
    private final DriveSystem driveSystem;
    private Matrix4d robotTransformFS;
    private final double robotInitialYaw;
    private Matrix4d robotTransformAS;
    private Matrix4d destinationFS;
    private final Map<Integer, Matrix4d> tagTransformsWS;
    private final RobotLogger logger;
    private final boolean imuEnabled;

    protected BasePilot(RobotLogger logger, DriveSystem driveSystem, Matrix4d allianceOriginTransform,
                       Matrix4d initialRobotTransform, AprilTagLibrary tagLibrary, boolean imuEnabled) {
        this.logger = logger;
        this.driveSystem = driveSystem;
        robotTransformFS = new Matrix4d(initialRobotTransform);
        robotInitialYaw = MatrixMagic.getYaw(initialRobotTransform);
        robotTransformAS = new Matrix4d();
        robotTransformAS.setIdentity(); // in case destination is never set (advise mode)
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
        this.imuEnabled = imuEnabled;
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
        if (knownDetectedTags == 0) {
          return;
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
    public void updateWithIMU(IMU imu, double weight) {
        if (!imuEnabled) {
            throw new UnsupportedOperationException("IMU not enabled on this pilot.");
        }
        double imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double currentYaw = MatrixMagic.getYaw(robotTransformFS);
        Matrix3d newRotMatrix = new Matrix3d();
        newRotMatrix.rotZ((robotInitialYaw + imuYaw) * weight + currentYaw * (1 - weight));
        robotTransformFS.setRotation(newRotMatrix);
    }
    @Override
    public boolean tick() {
        updateWithASOffset();
        Matrix4d destinationRS = convertToRS(destinationFS);
        Vector3d destTranslationRS = new Vector3d();
        destinationRS.get(destTranslationRS);
        double destAngleRS = MatrixMagic.getYaw(destinationRS);
        if (destTranslationRS.lengthSquared() < Math.pow(DISTANCE_EPSILON, 2)
                && destAngleRS < ANGLE_EPSILON) {
            logger.log("Reached destination");
            driveSystem.halt();
            return true;
        } else {
            driveSystem.swivel(destinationRS, 1.0);
            driveSystem.exec(0.5);
            return false;
        }
    }
    @Override
    public void tickAdvise() {
        updateWithASOffset();
    }
    @Override
    public double getFieldSpaceYaw() {
        return MatrixMagic.getYaw(robotTransformFS);
    }
    private void updateWithASOffset() {
        Matrix4d newRobotTransformAS = driveSystem.getActionSpaceTransform();
        updateWithOffset(MatrixMagic.invMul(robotTransformAS, newRobotTransformAS));
        robotTransformAS = newRobotTransformAS;

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
        DecimalFormat fmt = new DecimalFormat("00.0000 ");
        Vector3d robotTranslationFS = new Vector3d();
        robotTransformFS.get(robotTranslationFS);
        double robotRotationFS = MatrixMagic.getYaw(robotTransformFS);
        String robotTranslationFSOutput = fmt.format(robotTranslationFS.x) +
                fmt.format(robotTranslationFS.y) +
                fmt.format(robotTranslationFS.z);
        telemetry.addData("Robot position (FS meters)", robotTranslationFSOutput);
        telemetry.addData("Robot rotation (FS radians)", fmt.format(robotRotationFS));

        Vector3d destinationTranslationFS = new Vector3d();
        destinationFS.get(destinationTranslationFS);
        double destinationRotationFS = MatrixMagic.getYaw(destinationFS);
        String destinationTranslationFSOutput = fmt.format(destinationTranslationFS.x) +
                fmt.format(destinationTranslationFS.y) +
                fmt.format(destinationTranslationFS.z);
        telemetry.addData("Destination translation (FS meters)",
                destinationTranslationFSOutput);
        telemetry.addData("Destination rotation (FS radians)",
                fmt.format(destinationRotationFS));

        StringBuffer destinationTranslationRSOutput = new StringBuffer();
        Vector3d destinationTranslationRS = new Vector3d();
        Matrix4d destinationRS = convertToRS(destinationFS);
        destinationRS.get(destinationTranslationRS);
        double destinationRotationRS = MatrixMagic.getYaw(destinationRS);
        destinationTranslationRSOutput.append(fmt.format(destinationTranslationRS.x));
        destinationTranslationRSOutput.append(fmt.format(destinationTranslationRS.y));
        destinationTranslationRSOutput.append(fmt.format(destinationTranslationRS.z));
        telemetry.addData("Destination translation (RS meters)",
                destinationTranslationRSOutput.toString());
        telemetry.addData("Destination rotation (RS radians)",
                fmt.format(destinationRotationRS));

        StringBuffer robotTranslationASOutput = new StringBuffer();
        Vector3d robotTranslationAS = new Vector3d();
        driveSystem.getActionSpaceTransform().get(robotTranslationAS);
        double robotRotationAS = MatrixMagic.getYaw(driveSystem.getActionSpaceTransform());
        robotTranslationASOutput.append(fmt.format(robotTranslationAS.x));
        robotTranslationASOutput.append(fmt.format(robotTranslationAS.y));
        robotTranslationASOutput.append(fmt.format(robotTranslationAS.z));
        telemetry.addData("Robot translation (AS meters)", robotTranslationASOutput.toString());
        telemetry.addData("Robot rotation (AS radians)", fmt.format(robotRotationAS));
    }
}
