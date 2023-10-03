package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;

@TeleOp(name="Motor Test Linear OpMode", group="Linear OpMode")
public class TestMotorOpMode_Linear extends LinearOpMode {
	private String[] motorNames = {
		"left_front_drive",
		"left_back_drive",
		"right_front_drive",
		"right_back_drive"
	};
	private ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
	private DcMotor currentMotor = null;
	private DcMotor currentMotorIdx = 0;
	@Override
	public void runOpMode() {
		for (int i = 0; i < motorNames.size(); ++i) {
			motors[i] = 
