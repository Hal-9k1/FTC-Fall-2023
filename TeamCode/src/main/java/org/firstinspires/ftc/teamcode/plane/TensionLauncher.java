package org.firstinspires.ftc.teamcode.plane;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.input.PlaneInputInfo;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

public class TensionLauncher implements PlaneLauncher {
  private static final String SERVO_NAME = "plane_launcher_servo";
  private final Servo servo;
  private boolean launched;

  public TensionLauncher(HardwareMap hardwareMap) {
    servo = hardwareMap.get(Servo.class, SERVO_NAME);
    launched = false;
  }
  public Matrix4d getLaunchTransform() {
    Matrix4d transform = new Matrix4d();
    transform.setIdentity();
    transform.setTranslation(new Vector3d(1.0, 2.0, 0.0));
    return transform;
  }
  public void launch() {
    if (launched) return;
    launched = true;
    servo.setPosition(0.5);
  }
  public void tickInput(PlaneInputInfo inputInfo) {
    if (inputInfo.getShouldLaunch()) {
      launch();
    }
  }
}
