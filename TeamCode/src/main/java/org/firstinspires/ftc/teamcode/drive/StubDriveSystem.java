package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.input.DriveInputInfo;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector2d;

public class StubDriveSystem implements DriveSystem {
    @Override
    public void move(Vector2d direction, double speed) {}

    @Override
    public void turn(double angle, double speed) {}

    @Override
    public void swivel(Matrix4d transform, double speed) {}

    @Override
    public void halt() {}

    @Override
    public void exec() {}

    @Override
    public void tickInput(DriveInputInfo input) {}

    @Override
    public void startNewAction() {}

    @Override
    public Matrix4d getActionSpaceTransform() {
        Matrix4d identity = new Matrix4d();
        identity.setIdentity();
        return identity;
    }

    @Override
    public double getRobotBoundingRadius() {
        return 0;
    }
}
