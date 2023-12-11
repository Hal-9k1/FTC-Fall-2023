package org.firstinspires.ftc.teamcode.input;

/**
 * Input info sufficient to control a plane/drone launcher.
 */
public interface PlaneInputInfo {
  /**
   * Gets whether the launcher is currently being signalled to launch.
   * @return Whether the launcher is being signalled to launch.
   */
  boolean getShouldLaunch();
}
