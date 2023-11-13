package org.firstinspires.ftc.teamcode;

import javax.vecmath.Matrix4d;

public class MatrixMagic {
  public static double getYaw(Matrix4d mat) {
    // From this person online: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
    return Math.atan2(-mat.m20, mat.m00);
  }
  public static Matrix4d invMul(Matrix4d toInvert, Matrix4d toMultiply) {
    Matrix4d mat = new Matrix4d();
    mat.invert(toInvert);
    mat.mul(toMultiply);
    return mat;
  }
}
