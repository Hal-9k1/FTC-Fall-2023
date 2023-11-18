package org.firstinspires.ftc.teamcode.plane;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

public class TensionLauncher implements PlaneLauncher {
  private static final String SERVO_NAME = "SERVO_NAME_HERE";
  private final Servo servo;

  public TensionLauncher(HardwareMap hardwareMap) {
    servo = hardwareMap.get(Servo.class, SERVO_NAME);
  }
  public Matrix4d getLaunchTransform() {
    Matrix4d transform = new Matrix4d();
    transform.setIdentity();
    transform.setTranslation(new Vector3d(1.0, 2.0, 0.0));
    return transform;
  }
  public void launch() {
    servo.setPosition(Math.PI / 2.0);
  }
}
