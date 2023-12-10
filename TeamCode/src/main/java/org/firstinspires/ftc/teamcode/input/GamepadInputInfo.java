package org.firstinspires.ftc.teamcode.input;

/* pkg private */ class GamepadInputInfo implements DriveInputInfo, PlaneInputInfo, ArmInputInfo {
  private double axial;
  private double lateral;
  private double yaw;
  private boolean shouldLaunchPlane;
  private boolean shouldArmActivate;
  private boolean shouldArmCancel;

  /* pkg private */ GamepadInputInfo(double axial,
                                     double lateral,
                                     double yaw,
                                     boolean shouldLaunchPlane,
                                     boolean shouldArmActivate,
                                     boolean shouldArmCancel) {
    this.axial = axial;
    this.lateral = lateral;
    this.yaw = yaw;
    this.shouldLaunchPlane = shouldLaunchPlane;
    this.shouldArmActivate = shouldArmActivate;
    this.shouldArmCancel = shouldArmCancel;
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
  public boolean getShouldArmActivate() {
    return shouldArmActivate;
  }

  @Override
  public boolean getShouldArmCancel() {
    return shouldArmCancel;
  }
}
