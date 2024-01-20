package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MatrixMagic;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.input.AdvisableDriveInputInfo;
import org.firstinspires.ftc.teamcode.input.OmniGamepadMapping;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import javax.vecmath.Matrix4d;

/**
 * Uses a pilot to adjust input from a mapping before passing it to a drive system.
 * Input is therefore relative to the driver station.
 */
@TeleOp(name="Advised Drive", group="Iterative OpMode")
public class AdvisedDriveTeleOpMode extends OpMode {
  private DriveSystem driveSystem;
  private RobotPilot pilot;
  private OmniGamepadMapping mapping;
  private ElapsedTime runtime;
  private TelemetryLogger logger;
  private static final double ALLIANCE_ANGLE = Math.PI / 2; // in field space radians. positive for blue
  private boolean useAdvisor;
  private boolean didToggleAdvisor;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new MecanumDriveSystem(hardwareMap);
    Matrix4d ftcOriginTransform = new Matrix4d();
    ftcOriginTransform.rotZ(Math.PI);
    Matrix4d initialRobotTransform = new Matrix4d();
    initialRobotTransform.setIdentity();
    useAdvisor = true;
    didToggleAdvisor = false;
    
    pilot = new SimplePilot(logger, driveSystem, ftcOriginTransform, initialRobotTransform,
      AprilTagGameDatabase.getCenterStageTagLibrary());
    mapping = new OmniGamepadMapping(gamepad1);

    telemetry.addData("Status", "Initialized");
    //matrixSanityCheck();
    telemetry.update();
    logger.setFlushMode(false);
  }
  private void matrixSanityCheck() {
    Matrix4d rot = new Matrix4d();
    rot.rotZ(Math.PI / 2);
    telemetry.addData("Sanity - pi/2", MatrixMagic.getYaw(rot));
    rot.rotZ(Math.PI);
    telemetry.addData("Sanity - pi", MatrixMagic.getYaw(rot));
    rot.rotZ(3 * Math.PI / 2);
    telemetry.addData("Sanity - 3pi/2", MatrixMagic.getYaw(rot));
    rot.rotZ(Math.PI * 2);
    telemetry.addData("Sanity - 2pi", MatrixMagic.getYaw(rot));
    rot.rotZ(-Math.PI / 2);
    telemetry.addData("Sanity - -pi/2", MatrixMagic.getYaw(rot));
    rot.rotZ(-Math.PI);
    telemetry.addData("Sanity - -pi", MatrixMagic.getYaw(rot));
    rot.rotZ(-3 * Math.PI / 2);
    telemetry.addData("Sanity - -3pi/2", MatrixMagic.getYaw(rot));
    rot.rotZ(-Math.PI * 2);
    telemetry.addData("Sanity - -2pi", MatrixMagic.getYaw(rot));
  }
  @Override
  public void start() {
    runtime = new ElapsedTime();
  }

  @Override
  public void loop() {
    pilot.tickAdvise();
    mapping.generateInput();
    boolean shouldToggleAdvisor = ((AdvisableDriveInputInfo)mapping.getInput()).getShouldToggleAdvisor();
    if (shouldToggleAdvisor && !didToggleAdvisor) {
      useAdvisor = !useAdvisor;
    }
    didToggleAdvisor = shouldToggleAdvisor;
    if (useAdvisor) {
      ((AdvisableDriveInputInfo) mapping.getInput()).adviseRotation(
              pilot.getFieldSpaceYaw() + ALLIANCE_ANGLE);
    }
    driveSystem.tickInput(mapping.getInput());
    telemetry.addData("Status", "Running");
    telemetry.addData("Runtime", runtime.toString());
    telemetry.addData("Robot yaw FS", pilot.getFieldSpaceYaw());
    logger.addTelemetry();
    telemetry.update();
  }
}
