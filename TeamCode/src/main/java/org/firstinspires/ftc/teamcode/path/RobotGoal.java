package org.firstinspires.ftc.teamcode.path;

import javax.vecmath.Matrix4d;

/**
 * Represents a goal for the robot's physical state.
 */
public class RobotGoal {
    private Matrix4d transform;
    /* pkg private */ RobotGoal(Matrix4d transform) {
        this.transform = new Matrix4d(transform);
    }

    /**
     * Gets the goal field space transform.
     * @return The goal transform for the robot in field space.
     */
    public Matrix4d getTransform() {
        return new Matrix4d(transform);
    }
    // TODO: arm state
}
