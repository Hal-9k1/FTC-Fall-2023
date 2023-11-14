package org.firstinspires.ftc.teamcode.input;

/* pkg private */ class GamepadInputInfo implements DriveInputInfo {
  private double axial;
  private double lateral;
  private double yaw;
  /* pkg private */ GamepadInputInfo(double axial, double lateral, double yaw) {
    this.axial = axial;
    this.lateral = lateral;
    this.yaw = yaw;
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
}
