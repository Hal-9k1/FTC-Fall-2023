package org.firstinspires.ftc.teamcode.plane;

import org.firstinspires.ftc.teamcode.input.PlaneInputInfo;

import javax.vecmath.Matrix4d;

/**
 * A mechanism to launch a drone, typically (though not necessarily) a paper airplane.
 */
public interface PlaneLauncher {
  /**
   * Gets the field space transform the robot should take before launching the plane.
   * @return The field space transform of the launch site.
   */
  Matrix4d getLaunchTransform();

  /**
   * Launches the plane.
   */
  void launch();

  /**
   * Processes input info to decide whether the plane should be launched in TeleOp.
   * @param inputInfo The input info that decides whether the plane should now be launched.
   */
  void tickInput(PlaneInputInfo inputInfo);

  /**
   * Updates the launcher.
   */
  void tick();
}
