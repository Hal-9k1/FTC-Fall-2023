package org.firstinspires.ftc.teamcode;

import javax.vecmath.Matrix4d;

public class MatrixMagic {
  public static double getYaw(Matrix4d mat) {
    // From this person online: https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
    return Math.atan2(-mat.m10, mat.m00);
  }
  public static Matrix4d invMul(Matrix4d toInvert, Matrix4d toMultiply) {
    Matrix4d mat = new Matrix4d();
    mat.invert(toInvert);
    mat.mul(toMultiply);
    return mat;
  }
}
