package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AngledHolonomicDriveSystem;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MotorActionState;

import java.util.ArrayDeque;
import java.util.Queue;

import javax.vecmath.Vector2d;

/**
 * Executes a queue of MotorActionStates without using a RobotNavigator or PathPlanner. Consequently
 * has no idea where it is, is blinder than blind, and will not even correct for unexpected encoder
 * offsets.
 */
@Autonomous(name="Basic Auto", group="Iterative OpMode")
public class BasicAutoOpMode_Iterative extends OpMode {
  private DriveSystem driveSystem;
  private MotorActionState motorState;
  private ElapsedTime runtime;
  private Queue<MotorActionState> motorStateQueue;
  private TelemetryLogger logger;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new AngledHolonomicDriveSystem(logger, hardwareMap);
    ArrayDeque<MotorActionState> stateArrayDeque = new ArrayDeque<>();
    stateArrayDeque.add(driveSystem.computeMove(new Vector2d(0.0, 1.0), 0.01));
    motorStateQueue = stateArrayDeque;

    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }
  @Override
  public void start() {
    logger.setFlushMode(false);
    runtime = new ElapsedTime();
    motorState = motorStateQueue.poll();
  }
  @Override
  public void loop() {
    if (motorState != null && driveSystem.tick(motorState)) {
      motorState = motorStateQueue.poll();
    }
    if (motorState == null) {
      telemetry.addData("Status", "Finished");
      driveSystem.halt();
    } else {
      telemetry.addData("Status", "Running");
      telemetry.addData("Runtime", runtime.toString());
    }
    logger.addTelemetry();
    telemetry.update();
  }
}
