package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.AngledHolonomicDriveSystem;
import org.firstinspires.ftc.teamcode.input.DefaultGamepadMapping;

@TeleOp(name="Drive", group="Iterative OpMode")
public class DriveOpMode_Iterative extends OpMode {
  private DriveSystem driveSystem;
  private DefaultGamepadMapping mapping;
  private ElapsedTime runtime;

  @Override
  public void init() {
    driveSystem = new AngledHolonomicDriveSystem(hardwareMap);
    mapping = new DefaultGamepadMapping(gamepad1);

    telemetry.addData("Status", "Initialized");
    telemetry.update();
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
    telemetry.update();
  }
}
