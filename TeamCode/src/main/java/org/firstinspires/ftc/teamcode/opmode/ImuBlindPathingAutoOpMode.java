package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.navigator.BeelineNavigator;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.path.BlindPathPlanner;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.teamcode.pilot.ImuPilot;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import javax.vecmath.Matrix4d;

/**
 * Uses a pilot, BeelineNavigator, and a planner to blindly execute preset goals.
 * Will not respond to game state, AprilTags, or obstacles, but technically implements the full
 * four-layer stack.
 * Uses the Control Hub's IMU to augment robot localization.
 */
@Autonomous(name="IMU Blind Pathing Auto", group="Iterative OpMode")
public class ImuBlindPathingAutoOpMode extends OpMode {
    private TelemetryLogger logger;
    private DriveSystem driveSystem;
    private RobotPilot pilot;
    private RobotNavigator navigator;
    private PathPlanner pathPlanner;
    private IMU imu;
    private ElapsedTime runtime;

    @Override
    public void init() {
        logger = new TelemetryLogger(telemetry);
        logger.setFlushMode(true);
        driveSystem = new MecanumDriveSystem(hardwareMap);
        Matrix4d initialRobotTransform = new Matrix4d();
        initialRobotTransform.setIdentity();
        Matrix4d ftcOriginTransform = new Matrix4d();
        ftcOriginTransform.rotZ(Math.PI);
        pilot = new ImuPilot(logger, driveSystem, ftcOriginTransform, initialRobotTransform,
                AprilTagGameDatabase.getCenterStageTagLibrary());
        navigator = new BeelineNavigator(logger, pilot);
        pathPlanner = new BlindPathPlanner(logger, navigator);
        imu = hardwareMap.get(IMU.class, "imu 1");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)
        ));
        imu.resetYaw();

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
        if (pathPlanner.tick()) {
            telemetry.addData("Status", "Finished");
        } else {
            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", runtime.toString());
        }
        pilot.updateWithIMU(imu);
        pilot.addTelemetry(telemetry);
        logger.addTelemetry();
        telemetry.update();
    }
}
