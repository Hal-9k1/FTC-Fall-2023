package org.firstinspires.ftc.teamcode.input;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Input info sufficient to operate a holonomic drive system and a plane launcher.
 * @see DriveInputInfo
 * @see PlaneInputInfo
 */
/* pkg private */ class GamepadInputInfo implements AdvisableDriveInputInfo, PlaneInputInfo {
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
  @Override
  public void adviseRotation(double angle) {
    Vector3d vec = new Vector3d(axial, lateral, 0);
    Matrix3d mat = new Matrix3d();
    mat.rotZ(-angle);
    mat.transform(vec);
    axial = vec.x;
    lateral = vec.y;
  }
}
