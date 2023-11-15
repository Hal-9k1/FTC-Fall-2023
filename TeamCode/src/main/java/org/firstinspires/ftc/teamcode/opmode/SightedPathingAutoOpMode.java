package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.navigator.BeelineNavigator;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.path.BlindPathPlanner;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.teamcode.vision.RobotEye;
import org.firstinspires.ftc.teamcode.vision.AprilRobotEye;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import javax.vecmath.Matrix4d;

/**
 * Uses a pilot, BeelineNavigator, and a BlindPathPlanner to blindly execute preset goals.
 * The planner does not react to game state, but the pilot can use the webcam mounted on the robot
 * to make better guesses about its current location and make course adjustments.
 */
@Autonomous(name="Sighted Pathing", group="Iterative OpMode")
public class SightedPathingAutoOpMode extends OpMode {
    private static final Matrix4d RED_ALLIANCE_ORIGIN;
    static {
        RED_ALLIANCE_ORIGIN = new Matrix4d();
        RED_ALLIANCE_ORIGIN.setIdentity();
    }
    private static final String WEBCAM_NAME = "WEBCAM NAME HERE";
    private TelemetryLogger logger;
    private DriveSystem driveSystem;
    private RobotPilot pilot;
    private RobotNavigator navigator;
    private PathPlanner pathPlanner;
    private ElapsedTime runtime;
    private Matrix4d cameraTransformRS;
    private RobotEye eye;

    @Override
    public void init() {
        logger = new TelemetryLogger(telemetry);
        logger.setFlushMode(true);
        driveSystem = new MecanumDriveSystem(hardwareMap);
        Matrix4d initialRobotTransform = new Matrix4d();
        initialRobotTransform.setIdentity();
        pilot = new SimplePilot(logger, driveSystem, RED_ALLIANCE_ORIGIN, initialRobotTransform
                AprilTagGameDatabase.getCenterStageTagLibrary());
        navigator = new BeelineNavigator(logger, pilot);
        pathPlanner = new BlindPathPlanner(logger, navigator);
        eye = new AprilRobotEye(hardwareMap.get(WebcamName.class, WEBCAM_NAME));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        logger.setFlushMode(false);
    }
    @Override
    public void start() {
        runtime = new ElapsedTime();
        cameraTransformRS = new Matrix4d();
        cameraTransformRS.setIdentity();
    }
    @Override
    public void loop() {
        if (pathPlanner.tick()) {
            telemetry.addData("Status", "Finished");
        } else {
            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", runtime.toString());
        }
        pilot.updateWithTags(cameraTransformRS, eye.getTagDetections());
        pilot.addTelemetry(telemetry);
        logger.addTelemetry();
        telemetry.update();
    }
}
