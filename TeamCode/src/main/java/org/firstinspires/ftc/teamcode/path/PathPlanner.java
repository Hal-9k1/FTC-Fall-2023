package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.drive.MotorActionState;

public interface PathPlanner {
  MotorActionState getNextAction();
  // TODO: add interface to tell the planner about field state
}
