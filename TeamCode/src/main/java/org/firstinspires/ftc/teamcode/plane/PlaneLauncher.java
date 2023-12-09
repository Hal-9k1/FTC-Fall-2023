package org.firstinspires.ftc.teamcode.plane;

import org.firstinspires.ftc.teamcode.input.PlaneInputInfo;

import javax.vecmath.Matrix4d;

public interface PlaneLauncher {
  Matrix4d getLaunchTransform();
  void launch();
  void tickInput(PlaneInputInfo inputInfo);
  void tick();
}
