package org.firstinspires.ftc.teamcode.path;

import javax.vecmath.Matrix4d;

public class RobotGoal {
    private Matrix4d transform;
    /* pkg private */ RobotGoal(Matrix4d transform) {
        this.transform = new Matrix4d(transform);
    }
    public Matrix4d getTransform() {
        return new Matrix4d(transform);
    }
    // TODO: arm state
}
