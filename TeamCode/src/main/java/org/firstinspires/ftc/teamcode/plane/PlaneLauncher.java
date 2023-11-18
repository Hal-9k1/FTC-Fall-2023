package org.firstinspires.ftc.teamcode.plane;

import javax.vecmath.Matrix4d;

public interface PlaneLauncher {
  Matrix4d getLaunchTransform();
  void launch();
}
