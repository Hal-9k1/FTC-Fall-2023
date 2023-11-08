package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.input.DriveInputInfo;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector2d;

public interface DriveSystem {
  /**
   * Queues motor speeds for a 2D move action without turning.
   * @param direction A normalized vector describing the direction to move. The units of the vector
   *                  should be meters. Positive values for x and y move left and forward
   *                  respectively.
   * @param speed The fraction of the maximum speed to move. The ratios between motor speeds will be
   *              preserved.
   */
  void move(Vector2d direction, double speed);

  /**
   * Queues motor speeds for a turn about the robot's center.
   * @param angle The angle in radians to turn counterclockwise.
   * @param speed The fraction of the maximum speed to move. The ratios between motor speeds will be
   *              preserved.
   */
  void turn(double angle, double speed);
  void swivel(Matrix4d transform, double speed);
  void halt();
  void exec();
  void tickInput(DriveInputInfo input);
  void startNewAction();
  Matrix4d getActionSpaceTransform();
  double getFootprintRadius();
}
