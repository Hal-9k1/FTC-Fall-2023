package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MatrixMagic;
import org.firstinspires.ftc.teamcode.input.DriveInputInfo;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

/**
 * A DriveSystem using four squarely positioned Mecanum wheels on a rectangular chassis.
 */
public class MecanumDriveSystem implements DriveSystem {
  private static final String LEFT_FRONT_NAME = "left_front_drive";
  private static final String LEFT_BACK_NAME = "left_back_drive";
  private static final String RIGHT_FRONT_NAME = "right_front_drive";
  private static final String RIGHT_BACK_NAME = "right_back_drive";
  private static final double INITIAL_CALIBRATION_CONSTANT = -0.23;
  private static final String[] MOTOR_NAMES = {
    LEFT_FRONT_NAME,
    LEFT_BACK_NAME,
    RIGHT_FRONT_NAME,
    RIGHT_BACK_NAME
  };
  private static final double WHEEL_SPAN_METERS = 0.58;
  private static final double HALF_WHEEL_SPAN = WHEEL_SPAN_METERS / 2.0;
  private static final double BOUNDING_RADIUS = HALF_WHEEL_SPAN + 0.04;
  private static final Map<String, Double> GEARBOX_RATIOS = new HashMap<>();
  private static final Map<String, Double> WHEEL_RADII_METERS = new HashMap<>();
  private static final Map<String, Double> REV_LENGTHS = new HashMap<>();
  private static final Map<String, Double> CALIBRATION = new HashMap<>();
  static {
    // input shaft rotation / output shaft rotation
    GEARBOX_RATIOS.put(LEFT_FRONT_NAME, 1.0);
    GEARBOX_RATIOS.put(LEFT_BACK_NAME, 1.0);
    GEARBOX_RATIOS.put(RIGHT_FRONT_NAME, 1.0);
    GEARBOX_RATIOS.put(RIGHT_BACK_NAME, 1.0);

    WHEEL_RADII_METERS.put(LEFT_FRONT_NAME, 0.04);
    WHEEL_RADII_METERS.put(LEFT_BACK_NAME, 0.04);
    WHEEL_RADII_METERS.put(RIGHT_FRONT_NAME, 0.04);
    WHEEL_RADII_METERS.put(RIGHT_BACK_NAME, 0.04);

    for (String name : MOTOR_NAMES) {
      REV_LENGTHS.put(name, 2.0 * Math.PI * WHEEL_RADII_METERS.get(name));
    }

    // how much the calibration constant affects the motion of each motor
    CALIBRATION.put(LEFT_FRONT_NAME, 0.0);
    CALIBRATION.put(LEFT_BACK_NAME, 1.0);
    CALIBRATION.put(RIGHT_FRONT_NAME, 0.0);
    CALIBRATION.put(RIGHT_BACK_NAME, 1.0);
  }

  private final Map<String, DcMotor> motors;
  private final Map<String, Double> ticksPerRev;
  private final Map<String, Double> actionInitEncs;
  private final Map<String, Double> pendingMotorPowers;
  private double calibrationConstant;

  public MecanumDriveSystem(HardwareMap hardwareMap) {
    motors = new HashMap<>();
    ticksPerRev = new HashMap<>();
    actionInitEncs = new HashMap<>();
    pendingMotorPowers = new HashMap<>();
    calibrationConstant = INITIAL_CALIBRATION_CONSTANT;
    for (String name : MOTOR_NAMES) {
      DcMotor motor = hardwareMap.get(DcMotor.class, name);
      motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motors.put(name, motor);
      ticksPerRev.put(name, motor.getMotorType().getTicksPerRev() * GEARBOX_RATIOS.get(name));
      actionInitEncs.put(name, 0.0);
      pendingMotorPowers.put(name, 0.0);
    }
    motors.get(LEFT_FRONT_NAME).setDirection(DcMotorSimple.Direction.REVERSE);
    motors.get(LEFT_BACK_NAME).setDirection(DcMotorSimple.Direction.REVERSE);
    motors.get(RIGHT_FRONT_NAME).setDirection(DcMotorSimple.Direction.FORWARD);
    motors.get(RIGHT_BACK_NAME).setDirection(DcMotorSimple.Direction.REVERSE);

    startNewAction();
  }
  @Override
  public void move(Vector2d direction, double weight) {
    addToPendingPowers(calculateMotorPowers(direction.y * weight, direction.x * weight, 0));
  }
  @Override
  public void turn(double angle, double weight) {
    addToPendingPowers(calculateMotorPowers(0, 0, HALF_WHEEL_SPAN * angle * weight));
  }
  @Override
  public void swivel(Matrix4d transform, double weight) {
    double yaw = MatrixMagic.getYaw(transform);
    Vector3d translation = new Vector3d();
    transform.get(translation);
    double turnLength = HALF_WHEEL_SPAN * yaw;
    addToPendingPowers(calculateMotorPowers(translation.x * weight, translation.y * weight,
      turnLength * weight));
  }
  @Override
  public void halt() {
    motors.forEach((k, v) -> v.setPower(0));
  }
  @Override
  public void tickInput(DriveInputInfo input) {
    Map<String, Double> powers = calculateMotorPowers(input.getDriveAxial(),
      input.getDriveLateral(), input.getDriveYaw());
    powers.replaceAll((k, v) -> v * (1.0 + calibrationConstant * CALIBRATION.get(k)));
    normalizePowers(powers);
    motors.forEach((k, v) -> v.setPower(powers.get(k)));
  }

  @Override
  public void exec(double speed) {
    pendingMotorPowers.replaceAll((k, v) -> v * (1.0 + calibrationConstant * CALIBRATION.get(k)));
    normalizePowers(pendingMotorPowers);
    motors.forEach((k, v) -> {
      v.setPower(pendingMotorPowers.get(k) * speed);
      pendingMotorPowers.put(k, 0.0);
    });
  }
  @Override
  public void startNewAction() {
    actionInitEncs.forEach((k, v) -> actionInitEncs.put(k,
      (double)motors.get(k).getCurrentPosition()));
  }
  @Override
  public Matrix4d getActionSpaceTransform() {
    Map<String, Double> dists = new HashMap<>();
    for (String name : MOTOR_NAMES) {
      dists.put(name, ((double)motors.get(name).getCurrentPosition()
        - actionInitEncs.get(name)) / ticksPerRev.get(name) * REV_LENGTHS.get(name));
    }
    // If lf points rf, lb points lf, rf points lf, rb points rf:
    // lf = a + l - y
    // lb = a - l - y
    // rf = a - l + y
    // rb = a + l + y
    // a = (lf + rf) / 2
    // l = (lf - lb) / 2
    // y = (rb - lf) / 2

    double axial = dists.values().stream().mapToDouble(x -> x).sum() / 4;
    double lateral = (dists.get(LEFT_FRONT_NAME) - dists.get(LEFT_BACK_NAME)
      - dists.get(RIGHT_FRONT_NAME) + dists.get(RIGHT_BACK_NAME)) / 4;
    double yaw = (dists.get(RIGHT_FRONT_NAME) + dists.get(RIGHT_BACK_NAME)
      - dists.get(LEFT_FRONT_NAME) - dists.get(LEFT_BACK_NAME)) / 4;
    //double axial = (dists.get(LEFT_FRONT_NAME) + dists.get(RIGHT_FRONT_NAME)) / 2;
    //double lateral = (dists.get(LEFT_FRONT_NAME) - dists.get(LEFT_BACK_NAME)) / 2;
    //double yaw = (dists.get(RIGHT_BACK_NAME) - dists.get(LEFT_FRONT_NAME)) / 2;
    Matrix3d yawMat = new Matrix3d();
    yawMat.rotZ(yaw / HALF_WHEEL_SPAN);
    return new Matrix4d(yawMat, new Vector3d(axial, lateral, 0.0), 1.0);
  }
  @Override
  public double getRobotBoundingRadius() {
    return BOUNDING_RADIUS;
  }

  /**
   * Adjusts the implementation-defined calibration constant. Calling code has no way to evaluate
   * the fitness of the current value; a human observer is needed. Firmly not for production code.
   * @param delta The delta to change the constant by.
   */
  public void adjustCalibrationConstant(double delta) {
    calibrationConstant += delta;
  }

  /**
   * @param axial The distance in meters in the axial (front-back) direction. Positive values move
   *              the robot forward.
   * @param lateral The distance in meters in the lateral (left-right) direction. Positive values
   *                move the robot left.
   * @param yaw The number of revolutions to turn counterclockwise.
   */
  private Map<String, Double> calculateMotorPowers(double axial, double lateral, double yaw) {
    Map<String, Double> powers = new HashMap<>();
    // If lf points rf, lb points lf, rf points lf, rb points rf:
    powers.put(LEFT_FRONT_NAME, axial + lateral - yaw);
    powers.put(LEFT_BACK_NAME, axial - lateral - yaw);
    powers.put(RIGHT_FRONT_NAME, axial - lateral + yaw);
    powers.put(RIGHT_BACK_NAME, axial + lateral + yaw);
    powers.forEach((k, v) -> powers.put(k, v * ticksPerRev.get(k) / REV_LENGTHS.get(k)));
    return powers;
  }
  private void normalizePowers(Map<String, Double> powers) {
    // never increase powers during normalization
    double maxAbsEnc = Math.max(1.0, powers.values().stream().mapToDouble(v -> v)
      .map(Math::abs).max().getAsDouble());
    powers.forEach((k, v) -> powers.put(k, v / maxAbsEnc));
  }
  private void addToPendingPowers(Map<String, Double> powers) {
    pendingMotorPowers.forEach((k, v) -> pendingMotorPowers.put(k, v + powers.get(k)));
  }
}
