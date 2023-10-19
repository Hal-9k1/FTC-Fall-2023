package org.firstinspires.ftc.teamcode.drive;

import javax.vecmath.Vector2d;
import org.firstinspires.ftc.teamcode.drive.MotorActionState;
import org.firstinspires.ftc.teamcode.input.InputInfo;

public interface DriveSystem {
  MotorActionState computeMove(Vector2d direction, double speed);
  MotorActionState computeTurn(double angle, double speed);
  MotorActionState computeLinearSwivel(Vector2d direction, double angle, double speed);
  boolean tick(MotorActionState motorState);
  void tickInput(InputInfo input);
  Vector2d getUnexpectedOffset(MotorActionState motorState);
}
