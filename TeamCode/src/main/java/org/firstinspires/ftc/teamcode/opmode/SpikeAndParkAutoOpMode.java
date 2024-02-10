package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.logging.NopLogger;
import org.firstinspires.ftc.teamcode.navigator.BeelineNavigator;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.teamcode.path.SpikeAndParkPathPlanner;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import javax.vecmath.Matrix4d;

/**
 * Moves to the position of a the randomly placed spike, then parks backstage.
 * Uses BeelineNavigator and therefore requires a planner that effectively gives waypoints as goals.
 * (It can't actually see right now, so it'll get stuck waiting for spike transforms. It'll be way
 * more interesting after feature/vision is merged.)
 */
@Autonomous(name="Spike and Park Auto", group="Iterative OpMode")
public class SpikeAndParkAutoOpMode extends OpMode {
  private ElapsedTime runtime;
  private RobotLogger logger;
  private DriveSystem driveSystem;
  private RobotPilot pilot;
  private RobotNavigator navigator;
  private PathPlanner pathPlanner;
  private ElapsedTime runtime;
  @Override
  public void init() {
    logger = new NopLogger();
    driveSystem = new MecanumDriveSystem(hardwareMap);
    Matrix3d initialRobotRotationMat = new Matrix3d();
    initialRobotRotationMat.rotZ(Math.PI / 2);
    Matrix4d initialRobotTransform = new Matrix4d(initialRobotRotationMat,
      new Vector3d(1.5 * 0.61, -2.5 * 0.61, 0), 1.0);
    Matrix4d ftcOrigin = new Matrix4d();
    ftcOrigin.rotZ(Math.PI / 2);
    pilot = new SimplePilot(logger, driveSystem, initialRobotTransform, ftcOrigin,
      AprilTagGameDatabase.getCenterStageTagLibrary());
    navigator = new BeelineNavigator(logger, pilot);
    pathPlanner = new SpikeAndParkPathPlanner(logger, navigator);

    telemetry.addData("Status", "Initialized");
    telemetry.update();
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
