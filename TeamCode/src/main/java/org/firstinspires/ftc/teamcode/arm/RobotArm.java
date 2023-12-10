package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.input.ArmInputInfo;

public interface RobotArm {
    void raise();
    void reset();
    void tickInput(ArmInputInfo input);
    void tick();
}