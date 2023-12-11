package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.input.OmniGamepadMapping;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.plane.PlaneLauncher;
import org.firstinspires.ftc.teamcode.plane.TensionLauncher;

/**
 * Feeds input info from an input mapping to the drive and plane launching system.
 */
@TeleOp(name="Plane Drive", group="Iterative OpMode")
public class PlaneDriveTeleOpMode extends OpMode {
  private DriveSystem driveSystem;
  private PlaneLauncher planeLauncher;
  private OmniGamepadMapping mapping;
  private ElapsedTime runtime;
  private TelemetryLogger logger;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new MecanumDriveSystem(hardwareMap);
    planeLauncher = new TensionLauncher(logger, hardwareMap);
    mapping = new OmniGamepadMapping(gamepad1);

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
    mapping.generateInput();
    driveSystem.tickInput(mapping.getInput());
    planeLauncher.tickInput(mapping.getInput());
    //telemetry.addData("servo pos", hardwareMap.get(CRServo.class, "plane_launch_servo").);
    telemetry.addData("Status", "Running");
    telemetry.addData("Runtime", runtime.toString());
    logger.addTelemetry();
    telemetry.update();
  }
}
