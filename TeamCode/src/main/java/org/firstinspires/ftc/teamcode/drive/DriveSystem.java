package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.input.DriveInputInfo;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector2d;

public interface DriveSystem {
  /**
   * Computes the goal encoders for a 2D move action without turning.
   * @param direction The direction and magnitude to move. The units of the vector should be meters.
   *                  Positive values for x and y move left and forward respectively.
   * @param speed The fraction of the maximum speed to move. The ratios between motor speeds will be
   *              preserved.
   * @return A MotorActionState initialized to describe the move action.
   */
  MotorActionState computeMove(Vector2d direction, double speed);

  /**
   * Computes the goal encoders for a turn about the robot's center.
   * @param angle The angle in radians to turn counterclockwise.
   * @param speed The fraction of the maximum speed to move. The ratios between motor speeds will be
   *              preserved.
   * @return A MotorActionState initialized to describe the turn action.
   */
  MotorActionState computeTurn(double angle, double speed);
  MotorActionState computeLinearSwivel(Vector2d direction, double angle, double speed);
  MotorActionState computeLinearSwivel(Matrix4d transform, double speed);
  void halt();
  void init(MotorActionState motorState);
  boolean tick(MotorActionState motorState);
  void tickInput(DriveInputInfo input);
  Matrix4d getUnexpectedOffset(MotorActionState motorState);
  double getFootprintRadius();
}
