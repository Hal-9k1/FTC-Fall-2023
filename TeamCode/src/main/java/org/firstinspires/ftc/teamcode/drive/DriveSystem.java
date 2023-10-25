package org.firstinspires.ftc.teamcode.drive;

import javax.vecmath.Vector2d;
import javax.vecmath.Matrix4d;
import org.firstinspires.ftc.teamcode.drive.MotorActionState;
import org.firstinspires.ftc.teamcode.input.DriveInputInfo;

public interface DriveSystem {
  MotorActionState computeMove(Vector2d direction, double speed);
  MotorActionState computeTurn(double angle, double speed);
  MotorActionState computeLinearSwivel(Vector2d direction, double angle, double speed);
  MotorActionState computeLinearSwivel(Matrix4d transform, double speed);
  boolean tick(MotorActionState motorState);
  void tickInput(DriveInputInfo input);
  Matrix4d getUnexpectedOffset(MotorActionState motorState);
  double getFootprintRadius();
}
