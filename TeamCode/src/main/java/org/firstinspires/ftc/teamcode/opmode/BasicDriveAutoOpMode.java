package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MatrixMagic;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;

import java.util.ArrayDeque;
import java.util.Queue;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

/**
 * Executes a queue of swivels for fixed durations without using a pilot, navigator, or planner.
 * Consequently has no idea where it is, is blinder than blind, and will not even correct for
 * unexpected encoder offsets.
 */
@Autonomous(name="Basic Drive Auto", group="Iterative OpMode")
public class BasicDriveAutoOpMode extends OpMode {
  private DriveSystem driveSystem;
  private ElapsedTime runtime;
  private int stageNum;
  private Queue<Matrix4d> swivelQueue;
  private Matrix4d currentSwivel;
  private TelemetryLogger logger;
  private boolean finished;

  @Override
  public void init() {
    logger = new TelemetryLogger(telemetry);
    logger.setFlushMode(true);
    driveSystem = new MecanumDriveSystem(hardwareMap);
    finished = false;

    stageNum = -1;
    swivelQueue = new ArrayDeque<>();
    Matrix4d swivel;

    swivel = new Matrix4d();
    swivel.setIdentity();
    swivel.setTranslation(new Vector3d(0.0, 1.0, 0.0));
    swivelQueue.add(swivel);

    swivel = new Matrix4d();
    swivel.setIdentity();
    swivel.rotZ(Math.PI / 4);
    swivelQueue.add(swivel);

    swivel = new Matrix4d();
    swivel.setIdentity();
    swivel.rotZ(-Math.PI / 4);
    swivel.setTranslation(new Vector3d(0.0, 1.0, 0.0));
    swivelQueue.add(swivel);

    swivel = new Matrix4d();
    swivel.setIdentity();
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
    if (!finished && stageNum < (int)(runtime.milliseconds() / 2000)) {
      stageNum++;
      currentSwivel = swivelQueue.poll();
      if (currentSwivel == null) {
        finished = true;
        driveSystem.halt();
      } else {
        driveSystem.startNewAction();
      }
    }
    if (finished) {
      telemetry.addData("Status", "Finished");
    } else {
      driveSystem.swivel(currentSwivel, 1.0);
      driveSystem.exec(1.0);
      telemetry.addData("Status", "Running");
      telemetry.addData("Stage", stageNum);
      telemetry.addData("Current swivel yaw", MatrixMagic.getYaw(currentSwivel));
      telemetry.addData("Current swivel", "\n" + currentSwivel.toString());
      telemetry.addData("Runtime", runtime.toString());
    }
    logger.addTelemetry();
    telemetry.update();
  }
}
