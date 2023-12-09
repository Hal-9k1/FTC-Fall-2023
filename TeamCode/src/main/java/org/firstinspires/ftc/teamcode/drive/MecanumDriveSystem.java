package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.input.DriveInputInfo;

import java.util.Arrays;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public class MecanumDriveSystem implements DriveSystem {
  private static final String LEFT_FRONT_NAME = "left_front_drive";
  private static final String LEFT_BACK_NAME = "left_back_drive";
  private static final String RIGHT_FRONT_NAME = "right_front_drive";
  private static final String RIGHT_BACK_NAME = "right_back_drive";
  // input shaft rotation / output shaft rotation
  private static final double LEFT_FRONT_GEARBOX_RATIO = 1.0;
  private static final double LEFT_BACK_GEARBOX_RATIO = 1.0;
  private static final double RIGHT_FRONT_GEARBOX_RATIO = 1.0;
  private static final double RIGHT_BACK_GEARBOX_RATIO = 1.0;
  private static final String[] motorNames = {
    LEFT_FRONT_NAME,
    LEFT_BACK_NAME,
    RIGHT_FRONT_NAME,
    RIGHT_BACK_NAME
  };
  private static final double[] gearboxRatios = {
    LEFT_FRONT_GEARBOX_RATIO,
    LEFT_BACK_GEARBOX_RATIO,
    RIGHT_FRONT_GEARBOX_RATIO,
    RIGHT_BACK_GEARBOX_RATIO
  };
  private static final double WHEEL_RADIUS_METERS = 0.04;
  private static final double WHEELSPAN_METERS = 0.58;
  private static final double revLength = 2.0 * Math.PI * WHEEL_RADIUS_METERS;
  private static final double halfWheelspan = WHEELSPAN_METERS / 2.0;
  private final RobotLogger logger;
  private final DcMotor[] motors;
  private final double[] ticksPerRev;

  public MecanumDriveSystem(RobotLogger logger, HardwareMap hardwareMap) {
    this.logger = logger;
    motors = Arrays.stream(motorNames)
      .map(name -> hardwareMap.get(DcMotor.class, name))
      .toArray(DcMotor[]::new);
    motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
    motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    motors[2].setDirection(DcMotorSimple.Direction.FORWARD);
    motors[3].setDirection(DcMotorSimple.Direction.FORWARD);
    ticksPerRev = new double[4];
    for (int i = 0; i < 4; ++i) {
      ticksPerRev[i] = motors[i].getMotorType().getTicksPerRev() * gearboxRatios[i];
    }
  }
  @Override
  public MotorActionState computeMove(Vector2d direction, double speed) {
    return computeLinearSwivel(direction, 0, speed);
  }
  @Override
  public MotorActionState computeTurn(double angle, double speed) {
    return computeLinearSwivel(new Vector2d(), angle, speed);
  }
  @Override
  public MotorActionState computeLinearSwivel(Vector2d direction, double angle, double speed) {
    double turnLength = halfWheelspan * angle;
    // Corresponds to motorNames order.
    // If lf points rf, lb points lf, rf points lf, rb points rf:
    double[] finalEncoders = new double[4];
    double[] speeds = new double[4];
    calculateMotorPowers(direction.y / revLength,
      direction.x / revLength,
      turnLength / revLength,
      finalEncoders,
      speeds);
    Matrix3d angleMat = new Matrix3d();
    angleMat.rotZ(angle);
    MotorActionState.Builder builder = new MotorActionState.Builder(logger)
      .setGoalTransform(new Matrix4d(angleMat, new Vector3d(direction.x, direction.y, 0.0), 1.0));
    for (int i = 0; i < 4; ++i) {
      builder.setInitialEncoder(motorNames[i], 0)
        .setFinalEncoder(motorNames[i], finalEncoders[i])
        .setSpeed(motorNames[i], speeds[i] * speed);
    }
    return builder.build();
  }
  @Override
  public MotorActionState computeLinearSwivel(Matrix4d transform, double speed) {
    double yaw = Math.atan2(-transform.m20, transform.m00);
    Vector3d translation = new Vector3d();
    transform.get(translation);
    return computeLinearSwivel(new Vector2d(translation.x, translation.y), yaw, speed);
  }
  @Override
  public void halt() {
    for (DcMotor motor : motors) {
      motor.setPower(0);
    }
  }
  @Override
  public void init(MotorActionState motorState) {
    for (int i = 0; i < 4; ++i) {
      String name = motorNames[i];
      motorState.setInitialEncoder(name, motors[i].getCurrentPosition());
      motorState.setFinalEncoder(name, motorState.getInitialEncoder(name)
        + motorState.getFinalEncoder(name));
    }
  }
  @Override
  public boolean tick(MotorActionState motorState) {
    for (int i = 0; i < 4; ++i) {
      String name = motorNames[i];
      motors[i].setPower(motorState.getSpeed(name) * getSpeedFac(motorState.getAverageProgress()));
      motorState.setEncoder(name, motors[i].getCurrentPosition());
    }
    return motorState.getAverageProgress() >= 1.0;
  }

  private double getSpeedFac(double averageProgress) {
    return 3 * (averageProgress - Math.pow(averageProgress, 2)) + 0.250;
  }

  @Override
  public void tickInput(DriveInputInfo input) {
    double[] powers = new double[4];
    calculateMotorPowers(input.getDriveAxial(), input.getDriveLateral(), input.getDriveYaw(),
      new double[4], powers);
    for (int i = 0; i < 4; ++i) {
      motors[i].setPower(powers[i]);
    }
  }
  @Override
  public Matrix4d getUnexpectedOffset(MotorActionState motorState) {
    double[] offsets = new double[4]; // in meters
    double avgProgress = motorState.getAverageProgress();
    for (int i = 0; i < 4; ++i) {
      String name = motorNames[i];
      MotorConfigurationType type = motors[i].getMotorType();
      offsets[i] = (motorState.getProgress(name) - avgProgress)
              * (motorState.getFinalEncoder(name) - motorState.getInitialEncoder(name))
              / type.getTicksPerRev() * revLength;
    }
    // m0 = l + a - y
    // m1 = -l + a - y
    // m2 = -l + a + y
    // m3 = l + a + y
    // l = (m0 + m3) / 2
    // a = (m0 + m2) / 2
    // y = (m2 + m3) / 2
    double axial = (offsets[0] + offsets[2]) / 2;
    double lateral = (offsets[0] + offsets[3]) / 2 - axial;
    double yaw = (offsets[2] + offsets[3]) / 2 - axial;
    Matrix3d yawMat = new Matrix3d();
    yawMat.rotZ(yaw / (2.0 * Math.PI * halfWheelspan) );
    return new Matrix4d(yawMat, new Vector3d(lateral, axial, 0.0), 1.0);
  }
  @Override
  public double getFootprintRadius() {
    return halfWheelspan;
  }

  /**
   * @param axial The number of motor revolutions in the axial (front-back) direction.
   * @param lateral The number of motor revolutions in the lateral (left-right) direction.
   * @param yaw The number of revolutions to turn counterclockwise.
   * @param encoders An output array of size 4 that will hold the goal encoders for the drive
   *                 motion.
   * @param normalizedPowers An output array of size 4 that will hold motor powers (normalized if
   *                         needed from -1.0 to 1.0) for the drive motion.
   */
  private void calculateMotorPowers(double axial, double lateral, double yaw, double[] encoders,
    double[] normalizedPowers) {
//    logger.log("axial " + axial + " lateral " + lateral + " yaw " + yaw);
    double[] rawPowers = new double[4];
    rawPowers[0] = ticksPerRev[0] * (axial + lateral - yaw); // lf
    rawPowers[1] = ticksPerRev[1] * (axial - lateral - yaw); // lb
    rawPowers[2] = ticksPerRev[2] * (axial - lateral + yaw); // rf
    rawPowers[3] = ticksPerRev[3] * (axial + lateral + yaw); // rb
    System.arraycopy(rawPowers, 0, encoders, 0, 4);
//    logger.log("encoders " + encoders[0] + " " + encoders[1] + " " + encoders[2] + " "
//            + encoders[3]);
    // never increase powers during normalization
    double maxAbsEnc = Math.max(1.0, Arrays.stream(rawPowers).map(Math::abs).max().getAsDouble());
    normalizedPowers[0] = rawPowers[0] / maxAbsEnc;
    normalizedPowers[1] = rawPowers[1] / maxAbsEnc;
    normalizedPowers[2] = rawPowers[2] / maxAbsEnc;
    normalizedPowers[3] = rawPowers[3] / maxAbsEnc;
  }
}
