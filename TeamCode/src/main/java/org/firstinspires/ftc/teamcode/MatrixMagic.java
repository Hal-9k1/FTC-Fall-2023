package org.firstinspires.ftc.teamcode;

import javax.vecmath.Matrix4d;

public class MatrixMagic {
  public static double getYaw(Matrix4d mat) {
    // From this person online: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
    return Math.atan2(-mat.m20, mat.m00);
  }
}
