package org.firstinspires.ftc.teamcode;

import javax.vecmath.Matrix4d;

/**
 * Holds static helper methods useful for magic with matrices.
 */
public class MatrixMagic {
  /**
   * Extracts the Z axis rotation from the transformation matrix.
   * <p>
   * Extracts the Z rotation from mat such that:
   * <pre>{@code
   * Matrix4d mat = new Matrix4d();
   * mat.rotZ(rotation);
   * assert rotation == MatrixMagic.getYaw(mat); // works
   * }</pre>
   * @param mat The transformation matrix to extract the Z rotation from.
   */
  public static double getYaw(Matrix4d mat) {
    // From this person online: https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
    return Math.atan2(-mat.m10, mat.m00);
  }

  /**
   * Multiplies the inverse of one matrix by another.
   * <p>
   * Returns the product of the inverse of the first matrix and the second matrix:
   * A<sup>-1</sup> * B
   * @param toInvert The matrix whose inverse will be the left hand factor in the multiplication.
   * @param toMultiply The matrix that will be the right hand factor in the multiplication.
   * @return The product, as described above.
   */
  public static Matrix4d invMul(Matrix4d toInvert, Matrix4d toMultiply) {
    Matrix4d mat = new Matrix4d();
    mat.invert(toInvert);
    mat.mul(toMultiply);
    return mat;
  }
}
