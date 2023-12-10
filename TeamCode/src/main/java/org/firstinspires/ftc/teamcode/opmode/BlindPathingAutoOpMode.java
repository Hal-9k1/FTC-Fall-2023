package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.navigator.BeelineNavigator;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.path.BlindPathPlanner;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import javax.vecmath.Matrix4d;

/**
 * Uses a pilot, BeelineNavigator, and a planner to blindly execute preset goals.
 * Will not respond to game state, AprilTags, or obstacles, but technically implements the full
 * four-layer stack.
 */
@Autonomous(name="Blind Pathing Auto", group="Iterative OpMode")
public class BlindPathingAutoOpMode extends OpMode {
  private static final Matrix4d RED_ALLIANCE_ORIGIN;
  static {
    RED_ALLIANCE_ORIGIN = new Matrix4d();
    RED_ALLIANCE_ORIGIN.setIdentity();
  }
  private TelemetryLogger logger;
  private DriveSystem driveSystem;
  private RobotPilot pilot;
  private RobotNavigator navigator;
  private PathPlanner pathPlanner;
  private ElapsedTime runtime;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new MecanumDriveSystem(hardwareMap);
    Matrix4d initialRobotTransform = new Matrix4d();
    initialRobotTransform.setIdentity();
    pilot = new SimplePilot(logger, driveSystem, initialRobotTransform, RED_ALLIANCE_ORIGIN,
      AprilTagGameDatabase.getCenterStageTagLibrary());
    navigator = new BeelineNavigator(logger, pilot);
    pathPlanner = new BlindPathPlanner(logger, navigator);

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
    pilot.addTelemetry(telemetry);
    logger.addTelemetry();
    telemetry.update();
  }
}
