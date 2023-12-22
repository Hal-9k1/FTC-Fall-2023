package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MatrixMagic;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.StubDriveSystem;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.teamcode.vision.AprilRobotEye;
import org.firstinspires.ftc.teamcode.vision.RobotEye;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import javax.vecmath.Matrix4d;

/**
 * Feeds RobotEye input into a RobotPilot to test localization.
 */
@Autonomous(name="Vision Test", group="Iterative OpMode")
public class VisionTestAutoOpMode extends OpMode {
    private static final String WEBCAM_NAME = "Webcam 1";
    private DriveSystem driveSystem;
    private RobotPilot pilot;
    private RobotEye eye;
    private ElapsedTime runtime;
    private Matrix4d cameraTransformRS;
    private TelemetryLogger logger;

    @Override
    public void init() {
        logger = new TelemetryLogger(telemetry);
        logger.setFlushMode(true);
        driveSystem = new StubDriveSystem();
        Matrix4d initialRobotTransform = new Matrix4d();
        initialRobotTransform.setIdentity();
        // as defined by https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html:
        Matrix4d ftcOriginTransform = new Matrix4d();
        ftcOriginTransform.rotZ(Math.PI);
        pilot = new SimplePilot(logger, driveSystem, ftcOriginTransform, initialRobotTransform,
            AprilTagGameDatabase.getCenterStageTagLibrary());
        eye = new AprilRobotEye(hardwareMap.get(WebcamName.class, WEBCAM_NAME));

        cameraTransformRS = new Matrix4d();
        cameraTransformRS.setIdentity();

        Matrix4d testMat = new Matrix4d();
        testMat.rotZ(-Math.PI / 2);
        logger.log(Double.toString(MatrixMagic.getYaw(testMat)));
        testMat.rotZ(Math.PI / 2);
        logger.log(Double.toString(MatrixMagic.getYaw(testMat)));
        testMat.rotZ(-Math.PI);
        logger.log(Double.toString(MatrixMagic.getYaw(testMat)));
        testMat.rotZ(Math.PI);
        logger.log(Double.toString(MatrixMagic.getYaw(testMat)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        logger.setFlushMode(false);
    }
    @Override
    public void start() {
        runtime = new ElapsedTime();
    }
    @Override
    public void loop() {
        pilot.updateWithTags(cameraTransformRS, eye.getTagDetections());
        telemetry.addData("Status", "Running");
        telemetry.addData("Runtime", runtime);
        pilot.addTelemetry(telemetry); // the whole purpose of this OpMode is to run this line
        telemetry.update();
    }
}
