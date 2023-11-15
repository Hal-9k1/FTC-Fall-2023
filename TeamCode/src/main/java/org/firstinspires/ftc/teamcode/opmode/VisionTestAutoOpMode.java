package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.StubDriveSystem;
import org.firstinspires.ftc.teamcode.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.teamcode.vision.RobotEye;
import org.firstinspires.ftc.teamcode.vision.AprilRobotEye;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import javax.vecmath.Matrix4d;

@Autonomous(name="Vision Test", group="Iterative OpMode")
public class VisionTestAutoOpMode extends OpMode {
    private static final String WEBCAM_NAME = "Webcam 1";
    private DriveSystem driveSystem;
    private RobotPilot pilot;
    private RobotEye eye;
    private ElapsedTime runtime;
    private Matrix4d cameraTransformRS;
    private RobotLogger logger;

    @Override
    public void init() {
        logger = new TelemetryLogger(telemetry);
        driveSystem = new StubDriveSystem();
        Matrix4d initialRobotTransform = new Matrix4d();
        initialRobotTransform.setIdentity();
        pilot = new SimplePilot(driveSystem, initialRobotTransform,
            AprilTagGameDatabase.getCenterStageTagLibrary(), logger);
        eye = new AprilRobotEye(hardwareMap.get(WebcamName.class, WEBCAM_NAME));

        cameraTransformRS = new Matrix4d();
        cameraTransformRS.setIdentity();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
