package org.firstinspires.ftc.teamcode.input;

/**
 * Input info for drive that may be transformed by an "advised" rotation.
 */
public interface AdvisableDriveInputInfo extends DriveInputInfo {
  /**
   * Rotates the drive info.
   * @param angle The counterclockwise angle in radians to rotate the drive info by.
   */
  void adviseRotation(double angle);
  /**
   * Returns whether the control to toggle the advisor has been activated.
   * @return Whether the control to toggle the advisor has been activated.
   */
  boolean getShouldToggleAdvisor();
}
