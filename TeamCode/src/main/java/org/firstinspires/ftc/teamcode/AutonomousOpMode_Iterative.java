package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.teamcode.path.ObstaclelessPathPlanner;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.AngledHolonomicDriveSystem;

@Autonomous(name="Navigator", group="Iterative OpMode")
public class AutonomousOpMode_Iterative extends OpMode {
  private DriveSystem driveSystem;
  private RobotNavigator navigator;
  private PathPlanner pathPlanner;
  private MotorActionState motorState;
  private ElapsedTime runtime;

  @Override
  public void init() {
    driveSystem = new AngledHolonomicDriveSystem(hardwareMap);
    navigator = new SimpleNavigator();
    pathPlanner = new ObstaclelessPathPlanner();
    motorActions = pathPlanner.getNextAction();
    runtime = new ElapsedTime();

    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }
  @Override
  public void loop() {
    if (motorState != null && driveSystem.tick(motorState)) {
      motorState = pathPlanner.getNextAction();
    }
    if (motorState == null) {
      telemetry.addData("Status", "Finished");
    } else {
      navigator.updateWithTags(new ArrayList<AprilTagDetection>()); // TODO: detect tags
      navigator.updateWithOffset(driveSystem.calculateUnexpectedOffset(motorState));
      navigator.adjustMotion(actions);
      telemetry.addData("Status", "Running");
    }
    telemetry.update();
  }
}
