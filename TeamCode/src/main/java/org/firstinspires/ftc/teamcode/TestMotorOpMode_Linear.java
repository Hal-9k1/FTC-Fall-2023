package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Motor Test Linear OpMode", group="Linear OpMode")
public class TestMotorOpMode_Linear extends LinearOpMode {
  private List<String> motorNames = Arrays.asList(
    "left_front_drive",
    "left_back_drive",
    "right_front_drive",
    "right_back_drive"
    );
  private int[] encoderReadings;
  private int recordedEncoderReadings = 0;
  private int lastEncoderReading = 0;
  private int encoderReadingIdx = 0;
  private static final int ENCODER_READING_COUNT = 10000;
  private ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
  private DcMotor currentMotor = null;
  private int currentMotorIdx = 0;
  private boolean didChangeMotor = false;
  private boolean didResetEncReads = false;
  private static final float EPSILON = 0.001f;
  @Override
  public void runOpMode() {
    encoderReadings = new int[ENCODER_READING_COUNT];
    for (int i = 0; i < motorNames.size(); ++i) {
      motors.add(hardwareMap.get(DcMotor.class, motorNames.get(i)));
      motors.get(i).setDirection(DcMotorSimple.Direction.FORWARD);
    }
    updateCurrentMotor();
    waitForStart();
    while (opModeIsActive()) {
      boolean shouldChangeMotor = Math.abs(gamepad1.left_stick_y) > EPSILON;
      if (shouldChangeMotor && !didChangeMotor) {
        currentMotorIdx = (currentMotorIdx + 1) % motors.size();
        currentMotor.setPower(0.0f);
        updateCurrentMotor();
        resetEncoderReadings();
      }
      didChangeMotor = shouldChangeMotor;
      int encoderReading = currentMotor.getCurrentPosition();
      if (!didResetEncReads) {
        encoderReadings[encoderReadingIdx] = encoderReading - lastEncoderReading;
        encoderReadingIdx = (encoderReadingIdx + 1) % ENCODER_READING_COUNT;
        if (recordedEncoderReadings < ENCODER_READING_COUNT) {
          ++recordedEncoderReadings;
        }
      }
      lastEncoderReading = encoderReading;
      didResetEncReads = false;
      currentMotor.setPower(Math.abs(gamepad1.right_stick_y) > EPSILON ? 1.0f : 0.0f);

      telemetry.addData("cmidx", "Current motor index: %d",  currentMotorIdx);
      telemetry.addData("Current motor name", motorNames.get(currentMotorIdx));
      telemetry.addData("Current motor speed", "%4.2f ticks/sec", computeMotorSpeed());
      telemetry.addData("Sampled encoder readings", "%d", recordedEncoderReadings);
      telemetry.update();
    }
  }

  private void updateCurrentMotor() {
    currentMotor = motors.get(currentMotorIdx);
  }
  private void resetEncoderReadings() {
    recordedEncoderReadings = 0;
    encoderReadingIdx = 0;
    didResetEncReads = true;
  }
  private double computeMotorSpeed() {
    if (recordedEncoderReadings == 0) {
      return 0;
    }
    int total = 0;
    for (int i = 0; i < recordedEncoderReadings; ++i) {
      total += encoderReadings[i];
    }
    return (double)total / (double)recordedEncoderReadings;
  }
}
