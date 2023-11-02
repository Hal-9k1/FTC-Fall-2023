package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.input.DefaultGamepadMapping;

@TeleOp(name="Drive", group="Iterative OpMode")
public class DriveOpMode_Iterative extends OpMode {
  private DriveSystem driveSystem;
  private DefaultGamepadMapping mapping;
  private ElapsedTime runtime;
  private TelemetryLogger logger;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new MecanumDriveSystem(logger, hardwareMap);
    mapping = new DefaultGamepadMapping(gamepad1);

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
    driveSystem.tickInput(mapping.generateInput());
    telemetry.addData("Status", "Running");
    telemetry.addData("Runtime", runtime.toString());
    logger.addTelemetry();
    telemetry.update();
  }
}
