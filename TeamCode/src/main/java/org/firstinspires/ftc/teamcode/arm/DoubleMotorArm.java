package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.input.ArmInputInfo;
import org.firstinspires.ftc.teamcode.logging.RobotLogger;

public class DoubleMotorArm implements RobotArm {
    private static final String ARM_MOTOR_NAME = "arm_motor";
    private static final String ALT_ARM_MOTOR_NAME = "alt_arm_motor";
    private static final double RAISE_ANGLE = 120.0 / 180.0 * Math.PI;
    private static final double ARM_SPEED = 0.5;
    private static final double RETURN_SPEED = 1.0;
    private final DcMotor motor;
    private final DcMotor altMotor;
    private int goalPos;
    private int startPos;
    private int zeroPos;
    private boolean returning;
    private boolean stopped;
    private final RobotLogger logger;
    public DoubleMotorArm(HardwareMap hardwareMap, RobotLogger logger) {
        motor = hardwareMap.get(DcMotor.class, ARM_MOTOR_NAME);
        altMotor = hardwareMap.get(DcMotor.class, ALT_ARM_MOTOR_NAME);
        // they have to point the same direction:
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        altMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //altMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        zeroPos = motor.getCurrentPosition();
        this.logger = logger;
        returning = false;
    }

    @Override
    public void raise() {
        startPos = motor.getCurrentPosition();
        goalPos = zeroPos + (int)(RAISE_ANGLE / (2 * Math.PI) * motor.getMotorType().getTicksPerRev());
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
        if (input.getShouldArmCancel() && !returning) {
            reset();
        } else if (input.getShouldArmActivate()) { //&& stopped) {
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
        //telemetry.addData("arm status", "now: " + counter++ + " Arm delta: " + delta +" Returning: " + returning + " Stopped: " + stopped);
        if ((goalPos - startPos) * delta > 0) {
            double power = getMotorPower(delta);
            motor.setPower(power);
            altMotor.setPower(power);
        } else {
            motor.setPower(0.0);
            //altMotor.setPower(0.0);
            //if (!returning) {
            //    reset();
            //} else {
            //    returning = false;
            //    stopped = true;
            //}
            returning = false;
            stopped = true;
        }
    }

    private double getMotorPower(int deltaPos) {
        return Math.signum(deltaPos) * (returning ? RETURN_SPEED : ARM_SPEED);
    }
}
