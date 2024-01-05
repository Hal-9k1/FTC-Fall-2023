package org.firstinspires.ftc.teamcode.hook;

import org.firstinspires.ftc.teamcode.input.HookInputInfo;

public interface RobotHook {
  void tickInput(HookInputInfo input);
  void prime();
  void hook();

  void zero();

  void retreat();
  void tick();
}
