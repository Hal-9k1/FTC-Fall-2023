package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.input.OmniGamepadMapping;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Mecanum Weight Calibration")
public class MecanumCalibrationTeleOpMode extends OpMode {
    private static final double TRIGGER_EPSILON = 0.001;
    private MecanumDriveSystem driveSystem;
    private double calibrationConstant;
    private int deltaExp;
    private Map<String, Boolean> pressed;
    private OmniGamepadMapping mapping;
    @Override
    public void init() {
        driveSystem = new MecanumDriveSystem(hardwareMap);
        calibrationConstant = 0.0;
        deltaExp = -2;
        pressed = new HashMap<>();
        mapping = new OmniGamepadMapping(gamepad1);
        pressed.put("rb", Boolean.FALSE);
        pressed.put("rt", Boolean.FALSE);
        pressed.put("lb", Boolean.FALSE);
        pressed.put("lt", Boolean.FALSE);
        telemetry.addData("Status", "initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper && !pressed.get("rb").booleanValue()) {
            double delta = Math.pow(10, deltaExp);
            calibrationConstant += delta;
            driveSystem.adjustCalibrationConstant(delta);
        }
        pressed.put("rb", gamepad1.right_bumper);

        if (Math.abs(gamepad1.right_trigger) > TRIGGER_EPSILON && !pressed.get("rt").booleanValue()) {
            double delta = -Math.pow(10, deltaExp);
            calibrationConstant += delta;
            driveSystem.adjustCalibrationConstant(delta);
        }
        pressed.put("rt", Math.abs(gamepad1.right_trigger) > TRIGGER_EPSILON);

        if (gamepad1.left_bumper && !pressed.get("lb").booleanValue()) {
            ++deltaExp;
        }
        pressed.put("lb", gamepad1.left_bumper);

        if (Math.abs(gamepad1.left_trigger) > TRIGGER_EPSILON && !pressed.get("lt").booleanValue()) {
            --deltaExp;
        }
        pressed.put("lt", Math.abs(gamepad1.left_trigger) > TRIGGER_EPSILON);

        driveSystem.tickInput(mapping.generateInput());

        telemetry.addData("Status", "running");
        telemetry.addData("Calibration constant", calibrationConstant);
        telemetry.addData("Constant adjustment magnitude", deltaExp);
        telemetry.update();
    }
}
