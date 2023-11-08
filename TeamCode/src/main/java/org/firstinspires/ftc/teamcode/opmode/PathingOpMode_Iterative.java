package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.navigator.SimpleNavigator;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.teamcode.path.BlindPathPlanner;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import javax.vecmath.Matrix4d;

@Autonomous(name="Unplanned Pathing Auto", group="Iterative OpMode")
public class PathingOpMode_Iterative extends OpMode {
  private TelemetryLogger logger;
  private DriveSystem driveSystem;
  private RobotPilot pilot;
  private RobotNavigator navigator;
  private PathPlanner pathPlanner;
  private ElapsedTime runtime;
  private Matrix4d cameraTransformRS;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new MecanumDriveSystem(hardwareMap);
    Matrix4d initialRobotTransform = new Matrix4d();
    initialRobotTransform.setIdentity();
    pilot = new SimplePilot(driveSystem, initialRobotTransform,
      AprilTagGameDatabase.getCenterStageTagLibrary());
    navigator = new SimpleNavigator();
    pathPlanner = new BlindPathPlanner(navigator);

    telemetry.addData("Status", "Initialized");
    telemetry.update();
    logger.setFlushMode(false);
  }
  @Override
  public void start() {
    runtime = new ElapsedTime();
    motorState = pathPlanner.getNextAction();
    cameraTransformRS = new Matrix4d();
    cameraTransformRS.setIdentity();
  }
  @Override
  public void loop() {

    logger.addTelemetry();
    telemetry.update();
  }
}
