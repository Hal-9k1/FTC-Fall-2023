package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.input.ArmInputInfo;
import org.firstinspires.ftc.teamcode.logging.RobotLogger;

public class SingleMotorArm implements RobotArm {
    private static final String ARM_MOTOR_NAME = "arm_motor";
    private static final double RAISE_ANGLE = Math.PI / 2;
    private final DcMotor motor;
    private int goalPos;
    private int startPos;
    private int zeroPos;
    private boolean returning;
    private boolean stopped;
    private final RobotLogger logger;
    public SingleMotorArm(HardwareMap hardwareMap, RobotLogger logger) {
        motor = hardwareMap.get(DcMotor.class, ARM_MOTOR_NAME);
        zeroPos = motor.getCurrentPosition();
        this.logger = logger;
        returning = false;
    }

    @Override
    public void raise() {
        startPos = motor.getCurrentPosition();
        goalPos = zeroPos + (int)(RAISE_ANGLE / Math.PI * motor.getMotorType().getTicksPerRev());
        stopped = false;
    }

    @Override
    public void reset() {
        startPos = motor.getCurrentPosition();
        goalPos = zeroPos;
        returning = true;
        stopped = false;
    }

    @Override
    public void tickInput(ArmInputInfo input) {
        if (input.getShouldArmCancel()) {
            reset();
        } else if (input.getShouldArmActivate()) {
            raise();
        }
        tick();
    }

    @Override
    public void tick() {
        if (stopped) {
            return;
        }
        int delta = goalPos - motor.getCurrentPosition();
        if ((goalPos - startPos) * delta > 0) {
            motor.setPower(getMotorPower(delta));
        } else {
            motor.setPower(0.0);
            if (!returning) {
                reset();
            } else {
                returning = false;
                stopped = true;
            }
        }
    }

    private double getMotorPower(int deltaPos) {
        return Math.signum(deltaPos);
    }
}