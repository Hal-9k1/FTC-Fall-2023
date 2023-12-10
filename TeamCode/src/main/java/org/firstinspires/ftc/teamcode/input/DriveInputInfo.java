package org.firstinspires.ftc.teamcode.input;

/**
 * Input info sufficient to control an omnidirectional drive system.
 */
public interface DriveInputInfo {
  /**
   * Gets the axial input proportional to the other drive inputs.
   * @return The axial input, positive values meaning forward.
   */
  double getDriveAxial();
  /**
   * Gets the lateral input proportional to the other drive inputs.
   * @return The lateral input, positive values meaning right.
   */
  double getDriveLateral();
  /**
   * @return
   */
  double getDriveYaw();
}
