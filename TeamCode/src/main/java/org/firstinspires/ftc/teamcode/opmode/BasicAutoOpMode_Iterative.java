package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;

import java.util.ArrayDeque;
import java.util.Queue;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

/**
 * Executes a queue of MotorActionStates without using a RobotNavigator or PathPlanner. Consequently
 * has no idea where it is, is blinder than blind, and will not even correct for unexpected encoder
 * offsets.
 */
@Autonomous(name="Basic Auto", group="Iterative OpMode")
public class BasicAutoOpMode_Iterative extends OpMode {
  private DriveSystem driveSystem;
  private ElapsedTime runtime;
  private int stageNum;
  private Queue<Matrix4d> swivelQueue;
  private Matrix4d currentSwivel;
  private TelemetryLogger logger;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new MecanumDriveSystem(hardwareMap);

    stageNum = 0;
    swivelQueue = new ArrayDeque<>();
    Matrix4d swivel = new Matrix4d();
    swivel.setTranslation(new Vector3d(0.0, 1.0, 0.0));
    swivelQueue.add(swivel);
    swivel = new Matrix4d();
    swivel.rotZ(Math.PI / 4);
    swivelQueue.add(swivel);
    swivel = new Matrix4d();
    swivel.rotZ(-Math.PI / 4);
    swivel.setTranslation(new Vector3d(0.0, 1.0, 0.0));
    swivelQueue.add(swivel);
    swivel = new Matrix4d();
    swivel.setTranslation(new Vector3d(-2.0, -2.0, 0.0));
    swivelQueue.add(swivel);

    logger.setFlushMode(false);
    telemetry.update();
    telemetry.addData("Status", "Initialized");
    logger.addTelemetry();
    telemetry.update();
  }
  @Override
  public void start() {
    runtime = new ElapsedTime();
  }
  @Override
  public void loop() {
    if (stageNum < (int)(runtime.milliseconds() / 1000)) {
      stageNum++;
      currentSwivel = swivelQueue.poll();
      driveSystem.startNewAction();
    }
    if (currentSwivel == null) {
      driveSystem.halt();
      telemetry.addData("Status", "Finished");
    } else {
      driveSystem.swivel(currentSwivel, 1.0);
      driveSystem.exec();
      telemetry.addData("Status", "Running");
      telemetry.addData("Runtime", runtime.toString());
    }
    logger.addTelemetry();
    telemetry.update();
  }
}
