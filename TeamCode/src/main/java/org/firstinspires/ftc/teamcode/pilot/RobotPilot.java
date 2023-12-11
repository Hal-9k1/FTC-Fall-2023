package org.firstinspires.ftc.teamcode.pilot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import javax.vecmath.Matrix4d;

/**
 * Keeps track of the robot's field space transformation and moves the robot using a
 * {@link DriveSystem} to reach a given destination transformation.
 */
public interface RobotPilot {
  /**
   * Sets the destination transform the robot will move directly to.
   * @param destination The destination transform.
   */
  void setDestination(Matrix4d destination);

  /**
   * Localizes the robot using AprilTags.
   * <p>
   * Uses detections of AprilTags with known transformations to obtain an estimate of the robot's
   * own transformation.
   * @param cameraTransformRobotSpace The camera's transformation in robot space.
   * @param detections A list of AprilTagDetections to consider.
   */
  void updateWithTags(Matrix4d cameraTransformRobotSpace, List<AprilTagDetection> detections);

  /**
   * Localizes the robot using a direct offset.
   * <p>
   * Updates the pilot's understanding of the robot's transformation by an offset transformation.
   * @param offset The offset by which the robot's transformation should be updated.
   */
  void updateWithOffset(Matrix4d offset);

  /**
   * Updates the pilot.
   * <p>
   * Nudges the robot towards the current destination using the DriveSystem.
   * @return Whether the robot has reached the pilot's destination.
   */
  boolean tick();

  /**
   * Gets the radius of the smallest sphere that would completely enclose the robot.
   * @return The radius of the robot's bounding sphere.
   */
  double getRobotBoundingRadius();
  default void addTelemetry(Telemetry telemetry) {}
}
