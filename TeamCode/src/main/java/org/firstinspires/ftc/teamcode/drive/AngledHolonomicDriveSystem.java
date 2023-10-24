package org.firstinspires.ftc.teamcode.drive;

import java.util.Arrays;
import javax.vecmath.Vector2d;
import org.firstinspires.ftc.hardware.DcMotor;
import org.firstinspires.ftc.hardware.HardwareMap;

public class AngledHolonomicDriveSystem implements DriveSystem {
  private static final String LEFT_FRONT_NAME = "left_front_drive";
  private static final String LEFT_BACK_NAME = "left_back_drive";
  private static final String RIGHT_FRONT_NAME = "right_front_drive";
  private static final String RIGHT_BACK_NAME = "right_back_drive";
  private static final String[] motorNames = {
    LEFT_FRONT_NAME,
    LEFT_BACK_NAME,
    RIGHT_FRONT_NAME,
    RIGHT_BACK_NAME
  };
  private static final double WHEEL_RADIUS_METERS = 0.1;
  private static final double WHEELSPAN_METERS = 4.5;
  private static final double revLength = 2.0 * Math.PI * WHEEL_RADIUS_METERS;
  private static final double halfWheelspan = WHEELSPAN_METERS / 2.0;
  private DcMotor[] motors;

  public AngledHolonomicDriveSystem(HardwareMap hardwareMap) {
    motors = Arrays.stream(motorNames).map(name -> hardwareMap.get(DcMotor.class, name)).toArray();
  }
  
  public MotorActionState computeMove(Vector2d direction, double speed) {
    return computeLinearSwivel(direction, 0, speed);
  }
  public MotorActionState computeTurn(double angle, double speed) {
    return computeLinearSwivel(new Vector2d(), angle, speed);
  }
  public MotorActionState computeLinearSwivel(Vector2d direction, double angle, double speed) {
    double turnLength = halfWheelspan * angle;
    // Corrosponds to motorNames order.
    // If lf points rf, lb points lf, rf points lf, rb points rf:
    double[] finalEncoders = new double[4];
    double[] speeds = new double[4];
    calculateMotorPowers(direction.y, direction.x, turnLength, finalEncoders, speeds);
    Matrix3d angleMat = new Matrix3d();
    angleMat.rotZ(angle);
    MotorActionState.Builder builder = new MotorActionState.Builder()
      .setGoalTransform(new Matrix4d(angleMat, new Vector3d(direction.x, direction.y, 0.0)));
    for (int i = 0; i < 4; ++i) {
      builder.setInitialEncoder(motorNames[i], motors[i].getCurrentPosition())
        .setFinalEncoder(motorNames[i], finalEncoders[i])
        .setSpeed(motorNames[i], speeds[i]);
    }
    return builder.build();
  }
  boolean tick(MotorActionState motorState) {
    for (int i = 0; i < 4; ++i) {
      motors[i].setPower(motorState.getSpeed(motorNames[i]));
      double encoder = (double)motors[i].getCurrentPosition();
      double finalEncoder = motorState.getFinalEncoder(motorNames[i]);
      // Have we passed our goal yet?
      if (Math.signum((finalEncoder - encoder) * (finalEncoder - initialEncoder)) == -1) {
        return true;
      }
      motorState.setEncoder(encoder);
    }
    return false;
  }
  void tickInput(InputInfo input) {
    double[] powers = new double[4];
    calculateMotorPowers(input.driveAxial, input.driveLateral, input.driveYaw, new double[4],
      powers);
    for (int i = 0; i < 4; ++i) {
      motors[i].setPower(powers[i]);
    }
  }
  Matrix4d getUnexpectedOffset(MotorActionState motorState) {
    // m0 = l + a - y
    // m1 = -l - a - y
    // m2 = -l + a + y
    // m3 = l - a + y
    // l = (m0 + m3) / 2
    // a = (m0 + m2) / 2
    // y = (m2 + m3) / 2
    double[] progress = new double[4];
    for (int i = 0; i < 4; ++i) {
      String name = motorNames[i];
      encoders[i] = motorState.getEncoder(name);
      double initEnc = motorState.getInitialEncoder(name);
      progress[i] = (motorState.getEncoder(name) - initEnc)
        / (motorState.getFinalEncoder(name) - initEnc);
    }
    double[] offsets = new double[4]
    double avgProgress = Arrays.stream(progress).average().getAsDouble();
    for (int i = 0; i < 4; ++i) {
      String name = motorNames[i];
      double initEnc = motorState.getInitialEncoder(name);
      offsets[i] = (motorState.getEncoder(name) - initEnc)
        / (motorState.getFinalEncoder(name) - initEnc) - progress[i];
    }
    double lateral = (offsets[0] + offsets[3]) / 2;
    double axial = (offsets[0] + offsets[2]) / 2;
    double yaw = (offsets[2] + offsets[3]) / 2;
    Matrix3d yawMat = new Matrix3d();
    yawMat.rotZ(yaw);
    return new Matrix4d(yawMat, new Vector3d(lateral, axial, 0.0));
  }
  private void calculateMotorPowers(double axial, double lateral, double yaw, double[] rawPowers,
    double[] normalizedPowers) {
    rawPowers[0] =  lateral + axial - yaw; // lf
    rawPowers[1] = -lateral - axial - yaw; // lb
    rawPowers[2] = -lateral + axial + yaw; // rf
    rawPowers[3] =  lateral - axial + yaw; // rb
    double maxAbsEnc = Arrays.stream(rawPowers).map(x -> Math.abs(x)).max().getAsDouble();
    normalizedPowers[0] = rawPowers[0] / maxAbsEnc;
    normalizedPowers[1] = rawPowers[1] / maxAbsEnc;
    normalizedPowers[2] = rawPowers[2] / maxAbsEnc;
    normalizedPowers[3] = rawPowers[3] / maxAbsEnc;
  }
}
