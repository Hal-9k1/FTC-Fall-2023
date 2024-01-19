package org.firstinspires.ftc.teamcode.input;

/**
 * Input info for drive that may be transformed by an "advised" rotation.
 */
public interface AdviseableDriveInputInfo extends DriveInputInfo {
  /**
   * Rotates the drive info.
   * @param angle The counterclockwise angle in radians to rotate the drive info by.
   */
  void adviseRotation(double angle);
}
