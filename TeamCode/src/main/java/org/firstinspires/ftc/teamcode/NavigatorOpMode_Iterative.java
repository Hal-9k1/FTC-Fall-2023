package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.teamcode.path.ObstaclelessPathPlanner;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.AngledHolonomicDriveSystem;

@Autonomous(name="Navigator", group="Iterative OpMode")
public class NavigatorOpMode_Iterative extends OpMode {
  private DriveSystem driveSystem;
  private RobotNavigator navigator;
  private PathPlanner pathPlanner;
  private MotorActions motorActions;
  private ElapsedTime runtime;

  @Override
  public void init() {
    driveSystem = new AngledHolonomicDriveSystem();
    navigator = new SimpleNavigator();
    pathPlanner = new ObstaclelessPathPlanner();
    motorActions = pathPlanner.getNextAction();
    runtime = new ElapsedTime();

    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }
  @Override
  public void loop() {
    if (motorActions != null && driveSystem.tick(motorActions)) {
      motorActions = pathPlanner.getNextAction();
    }
    if (motorActions == null) {
      telemetry.addData("Status", "Finished");
    } else {
      navigator.updateWithTags(/* pass in AprilTagDetections */);
      navigator.updateWithOffset(driveSystem.calculateUnexpectedOffset(motorActions));
      navigator.adjustMotion(actions);
      telemetry.addData("Status", "Running");
    }
    telemetry.update();
  }
}
