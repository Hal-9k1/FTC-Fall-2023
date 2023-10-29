package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AngledHolonomicDriveSystem;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MotorActionState;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.navigator.SimpleNavigator;
import org.firstinspires.ftc.teamcode.path.BlindPathPlanner;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.ArrayList;

import javax.vecmath.Matrix4d;

@Autonomous(name="Blind Pathing Auto", group="Iterative OpMode")
public class PathingOpMode_Iterative extends OpMode {
  private DriveSystem driveSystem;
  private RobotNavigator navigator;
  private PathPlanner pathPlanner;
  private MotorActionState motorState;
  private ElapsedTime runtime;
  private Matrix4d cameraTransformRS;
  private TelemetryLogger logger;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new AngledHolonomicDriveSystem(logger, hardwareMap);
    Matrix4d initialRobotTransform = new Matrix4d();
    initialRobotTransform.setIdentity();
    navigator = new SimpleNavigator(initialRobotTransform,
      AprilTagGameDatabase.getCenterStageTagLibrary());
    pathPlanner = new BlindPathPlanner(driveSystem, navigator);

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
    if (motorState != null && driveSystem.tick(motorState)) {
      motorState = pathPlanner.getNextAction();
    }
    if (motorState == null) {
      telemetry.addData("Status", "Finished");
      driveSystem.halt();
    } else {
      navigator.updateWithTags(cameraTransformRS, new ArrayList<>()); // TODO: detect tags
      navigator.updateWithOffset(driveSystem.getUnexpectedOffset(motorState));
      navigator.adjustActions(motorState);
      driveSystem.tick(motorState);
      telemetry.addData("Status", "Running");
      telemetry.addData("Runtime", runtime.toString());
    }
    logger.addTelemetry();
    telemetry.update();
  }
}
