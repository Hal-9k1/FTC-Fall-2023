package org.firstinspires.ftc.teamcode.plane;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.input.PlaneInputInfo;
import org.firstinspires.ftc.teamcode.logging.RobotLogger;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

public class TensionLauncher implements PlaneLauncher {
  private static final String SERVO_NAME = "plane_launch_servo";
  private static final double RESET_TIME_MS = 4000.0;
  private RobotLogger logger;
  private final CRServo servo;
  private boolean launched;
  private boolean didReset;
  private ElapsedTime timeSinceLaunch;

  public TensionLauncher(RobotLogger logger, HardwareMap hardwareMap) {
    this.logger = logger;
    servo = hardwareMap.get(CRServo.class, SERVO_NAME);
    launched = false;
  }
  public Matrix4d getLaunchTransform() {
    Matrix4d transform = new Matrix4d();
    transform.setIdentity();
    transform.setTranslation(new Vector3d(1.0, 2.0, 0.0));
    return transform;
  }
  public void launch() {
    if (launched) {
      return;
    }
    timeSinceLaunch = new ElapsedTime();
    launched = true;
    servo.setPower(-1.0);
  }
  private void reset() {
    if (didReset) {
      return;
    }
    didReset = true;
    servo.setPower(0.0);
  }
  public void tickInput(PlaneInputInfo inputInfo) {
    if (inputInfo.getShouldLaunch()) {
      launch();
    }
    tick();
  }
  public void tick() {
    if (launched && timeSinceLaunch.milliseconds() > RESET_TIME_MS) {
      reset();
    }
  }
}
