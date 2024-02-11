package org.firstinspires.ftc.teamcode.input;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Input info sufficient to operate a holonomic drive system, a plane launcher, and an arm.
 * @see DriveInputInfo
 * @see PlaneInputInfo
 * @see ArmInputInfo
 */
/* pkg private */ class GamepadInputInfo implements AdvisableDriveInputInfo, PlaneInputInfo, ArmInputInfo {
  private double axial;
  private double lateral;
  private double yaw;
  private boolean shouldLaunchPlane;
  private boolean shouldToggleAdvisor;
  private boolean shouldArmActivate;
  private boolean shouldArmCancel;

  /* pkg private */ GamepadInputInfo(double axial,
                                     double lateral,
                                     double yaw,
                                     boolean shouldLaunchPlane,
                                     boolean shouldToggleAdvisor,
                                     boolean shouldArmActivate,
                                     boolean shouldArmCancel,
                                     boolean shouldLaunchPlane) {
    this.axial = axial;
    this.lateral = lateral;
    this.yaw = yaw;
    this.shouldLaunchPlane = shouldLaunchPlane;
    this.shouldToggleAdvisor = shouldToggleAdvisor;
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
  public boolean getShouldToggleAdvisor() {
    return shouldToggleAdvisor;
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
  @Override
  public boolean getShouldArmActivate() {
    return shouldArmActivate;
  }
  @Override
  public boolean getShouldArmCancel() {
    return shouldArmCancel;
  }
}
