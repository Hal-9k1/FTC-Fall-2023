package org.firstinspires.ftc.teamcode.input;

/**
 * Input info sufficient to operate a holonomic drive system and a plane launcher.
 * @see DriveInputInfo
 * @see PlaneInputInfo
 */
/* pkg private */ class GamepadInputInfo implements DriveInputInfo, PlaneInputInfo, HookInputInfo {
  private double axial;
  private double lateral;
  private double yaw;
  private boolean shouldLaunchPlane;
  private boolean shouldHook;
  private boolean shouldRetreatHook;

  /* pkg private */ GamepadInputInfo(double axial,
                                     double lateral,
                                     double yaw,
                                     boolean shouldLaunchPlane,
                                     boolean shouldHook,
                                     boolean shouldRetreatHook) {
    this.axial = axial;
    this.lateral = lateral;
    this.yaw = yaw;
    this.shouldLaunchPlane = shouldLaunchPlane;
    this.shouldHook = shouldHook;
    this.shouldRetreatHook = shouldRetreatHook;
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
  @Override
  public boolean getShouldHook() {
    return shouldHook;
  }
  @Override
  public boolean getShouldRetreatHook() {
    return shouldRetreatHook;
  }
}
