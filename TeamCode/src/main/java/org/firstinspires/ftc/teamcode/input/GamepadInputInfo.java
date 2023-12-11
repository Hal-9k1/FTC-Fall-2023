package org.firstinspires.ftc.teamcode.input;

/**
 * Input info sufficient to operate a holonomic drive system and a plane launcher.
 * @see DriveInputInfo
 * @see PlaneInputInfo
 */
/* pkg private */ class GamepadInputInfo implements DriveInputInfo, PlaneInputInfo {
  private double axial;
  private double lateral;
  private double yaw;
  private boolean shouldLaunchPlane;

  /* pkg private */ GamepadInputInfo(double axial,
                                     double lateral,
                                     double yaw,
                                     boolean shouldLaunchPlane) {
    this.axial = axial;
    this.lateral = lateral;
    this.yaw = yaw;
    this.shouldLaunchPlane = shouldLaunchPlane;
  }

  @Override
  public double getDriveAxial() {
    return axial;
  }
  @Override
  public double getDriveLateral() {
    return lateral;
  }
  @Override
  public double getDriveYaw() {
    return yaw;
  }
  @Override
  public boolean getShouldLaunch() {
    return shouldLaunchPlane;
  }
}
