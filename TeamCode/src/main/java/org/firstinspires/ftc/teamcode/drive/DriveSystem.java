package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.input.DriveInputInfo;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector2d;

/**
 * Powers motors to achieve movements in robot space. Specific to a particular configuration of
 * motors on a robot and the type of wheels used.
 */
public interface DriveSystem {
  /**
   * Queues motor speeds for a 2D move action without turning.
   * @param direction A normalized vector describing the direction to move. The units of the vector
   *                  should be meters. Positive values for x and y move left and forward
   *                  respectively.
   * @param weight The fraction of the maximum speed to move. The ratios between motor speeds will be
   *              preserved.
   */
  void move(Vector2d direction, double weight);

  /**
   * Queues motor speeds for a turn about the robot's center.
   * @param angle The angle in radians to turn counterclockwise.
   * @param weight The fraction of the maximum speed to move. The ratios between motor speeds will be
   *              preserved.
   */
  void turn(double angle, double weight);

  /**
   * Queues motor speeds for a combined turn and movement, specified by a transformation matrix.
   * @param transform The transformation matrix describing the motion.
   * @param weight The fraction of the maximum speed to move. The ratios between motor speeds will be
   *              preserved.
   */
  void swivel(Matrix4d transform, double weight);

  /**
   * Immediately brakes all motors. A call to {@code exec()} is not needed.
   */
  void halt();

  /**
   * Pushes the normalized sum of all queued motor speeds to the motors.
   */
  void exec(double speed);

  /**
   * Sets motor powers as required by the given input info.
   * @param input The input info that dictates the powers given to the motors.
   */
  void tickInput(DriveInputInfo input);

  /**
   * Marks the robot's current transform as the base for a new action. Subsequent calls to {@link
   * DriveSystem#getActionSpaceTransform()} will return transformations relative to this.
   */
  void startNewAction();

  /**
   * Gets the current transform of the robot relative to its transform at the beginning of the
   * current action (when {@link DriveSystem#startNewAction()} was last called).
   * @return The current transform of the robot in "action space" (relative to the robot's transform
   *         at the last call to {@code startNewAction()}).
   */
  Matrix4d getActionSpaceTransform();

  /**
   * Gets the radius of the bounding sphere that should represent the robot when pathfinding.
   * @return The radius in meters of the robot's bounding sphere.
   */
  double getRobotBoundingRadius();
}
