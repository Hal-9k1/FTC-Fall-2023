package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.logging.NopLogger;
import org.firstinspires.ftc.teamcode.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.navigator.BeelineNavigator;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.path.Alliance;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.teamcode.path.SpikeAndParkPathPlanner;
import org.firstinspires.ftc.teamcode.path.StartingPosition;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.teamcode.vision.RobotEye;
import org.firstinspires.ftc.teamcode.vision.SpikeAprilRobotEye;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.stream.Collectors;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

/**
 * Moves to the position of a the randomly placed spike, then parks backstage.
 * Uses BeelineNavigator and therefore requires a planner that effectively gives waypoints as goals.
 * (It can't actually see right now, so it'll get stuck waiting for spike transforms. It'll be way
 * more interesting after feature/vision is merged.)
 */
@Autonomous(name="Spike and Park Auto", group="Iterative OpMode")
public class SpikeAndParkAutoOpMode extends OpMode {
  private static final String WEBCAM_NAME = "Webcam 1";
  private static final double CAMERA_FOCAL_LENGTH_METERS = 0.033;
  // from https://stargazerslounge.com/topic/244964-cheap-astrophotography-galileoscope-and-logitech-c270/
  private static final double CAMERA_WIDTH_METERS = 0.00358;
  private static final double SPIKE_WIDTH_METERS = 0.0845;
  private ElapsedTime runtime;
  private RobotLogger logger;
  private DriveSystem driveSystem;
  private RobotPilot pilot;
  private RobotNavigator navigator;
  private PathPlanner pathPlanner;
  private RobotEye eye;
  private Matrix4d cameraTransformRS;
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
    pathPlanner = new SpikeAndParkPathPlanner(navigator, Alliance.BLUE, StartingPosition.FRONT);
    eye = new SpikeAprilRobotEye(hardwareMap.get(WebcamName.class, WEBCAM_NAME), SPIKE_WIDTH_METERS,
            CAMERA_FOCAL_LENGTH_METERS, CAMERA_WIDTH_METERS);
    Matrix3d cameraTransformRotationMat = new Matrix3d();
    cameraTransformRotationMat.setIdentity();
    Vector3d cameraTransformVecRS = new Vector3d(0.5 * 0.0254, -5.0625 * 0.0254, 9.875 * 0.0254);
    cameraTransformRS = new Matrix4d(cameraTransformRotationMat, cameraTransformVecRS, 1.0);

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
    pathPlanner.acceptSpikeTransforms(eye.getSpikeDetections().stream().map(m -> {
      Matrix4d n = new Matrix4d(cameraTransformRS);
      n.mul(m);
      return n;
    }).collect(Collectors.toList()));
    if (pathPlanner.tick()) {
      telemetry.addData("Status", "Finished");
    } else {
      telemetry.addData("Status", "Running");
      telemetry.addData("Runtime", runtime.toString());
    }
    pilot.addTelemetry(telemetry);
    //logger.addTelemetry();
    telemetry.update();
  }
}
