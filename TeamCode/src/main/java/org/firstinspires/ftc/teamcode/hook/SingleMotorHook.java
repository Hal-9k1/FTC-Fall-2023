package org.firstinspires.ftc.teamcode.hook;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.input.HookInputInfo;

public class SingleMotorHook implements RobotHook {
  // order is important. IN_PROGRESS, FINISHED
  private enum Status { ZEROING, ZERO, PRIMING, PRIMED, HOOKING, HOOKED };
  private static final double HOOKED_ANGLE = 45.0 / 180.0 * Math.PI;
  private static final double PRIMED_ANGLE = 115.0 / 180.0 * Math.PI;
  private static final double ZERO_ANGLE = 5.0 / 180.0 * Math.PI; // not actually zero!
  private static final String MOTOR_NAME = "hook_motor";
  private final DcMotor motor;
  private int goalPos;
  private int startPos;
  private int zeroPos;
  private Status status;
  private boolean hookWasPressed;
  private boolean retreatWasPressed;
  public SingleMotorHook(HardwareMap hardwareMap) {
    motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
    zeroPos = motor.getCurrentPosition();
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    status = Status.ZERO;
    hookWasPressed = false;
    retreatWasPressed = false;
  }
  public void tickInput(HookInputInfo input) {
    if (input.getShouldHook() && !hookWasPressed) {
      if (status == Status.ZERO) {
        prime();
      }
      if (status == Status.PRIMED) {
        hook();
      }
    } else if (input.getShouldRetreatHook() && !retreatWasPressed) {
      retreat();
    }
    hookWasPressed = input.getShouldHook();
    retreatWasPressed = input.getShouldRetreatHook();
    tick();
  }
  @Override
  public void prime() {
    setAngle(PRIMED_ANGLE);
    status = Status.PRIMING;
  }
  @Override
  public void hook() {
    setAngle(HOOKED_ANGLE);
    status = Status.HOOKING;
  }
  @Override
  public void zero() {
    setAngle(ZERO_ANGLE);
    status = Status.ZEROING;
  }
  @Override
  public void retreat() {
    if (status == Status.HOOKING) {
      prime();
    } else if (status == Status.PRIMING || status == Status.PRIMED) {
      zero();
    }
  }
  @Override
  public void tick() {
    if (status == Status.HOOKING || status == Status.PRIMING || status == Status.ZEROING) {
      int delta = goalPos - motor.getCurrentPosition();
      if ((goalPos - startPos) * delta > 0) { // signs are the same
        motor.setPower(getMotorPower(delta));
      } else { // signs are different, we've gone over goalPos
        motor.setPower(0.0);
        status = Status.values()[status.ordinal() + 1];
      }
    }
  }

  private void setAngle(double angle) {
    startPos = motor.getCurrentPosition();
    goalPos = (int)Math.round(angle / (2 * Math.PI) * motor.getMotorType().getTicksPerRev());
  }

  private double getMotorPower(int deltaPos) {
    return Math.signum(deltaPos) * 0.25; // could do some easing here
  }
  public void addTelemetry(Telemetry telemetry) {
    telemetry.addData("Hook status", status);
    telemetry.addData("Hook goal pos", goalPos);
  }
}
