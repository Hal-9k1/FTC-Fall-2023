package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

@TeleOp(name="Motor Test Linear OpMode", group="Linear OpMode")
public class TestMotorOpMode_Linear extends LinearOpMode {
	private List<String> motorNames = Arrays.asList(
		"left_front_drive",
		"left_back_drive",
		"right_front_drive",
		"right_back_drive"
    );
	private ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
	private DcMotor currentMotor = null;
	private int currentMotorIdx = 0;
  private boolean changedMotor = false;
  private static float EPSILON = 0.001f;
	@Override
	public void runOpMode() {
		for (int i = 0; i < motorNames.size(); ++i) {
      motors.add(hardwareMap.get(DcMotor.class, motorNames.get(i)));
    }
    updateCurrentMotor();
    waitForStart();
    while (opModeIsActive()) {
      boolean shouldChangeMotor = Math.abs(gamepad1.left_stick_y) > EPSILON;
      if (shouldChangeMotor && !changedMotor) {
        currentMotorIdx = (currentMotorIdx + 1) % motors.size();
        updateCurrentMotor();
      }
      changedMotor = shouldChangeMotor;
      currentMotor.setPower(Math.abs(gamepad1.right_stick_y) > EPSILON ? 1.0f : 0.0f);

      //telemetry.addData("cmidx", "Current motor index: %2.0d",  currentMotorIdx);
      telemetry.addData("Current motor name", motorNames.get(currentMotorIdx));
      telemetry.addData("Gamepad 1 left stick x", "%4.2f", gamepad1.left_stick_x);
      telemetry.update();
    }
  }

  public void updateCurrentMotor() {
    currentMotor = motors.get(currentMotorIdx);
  }
}
