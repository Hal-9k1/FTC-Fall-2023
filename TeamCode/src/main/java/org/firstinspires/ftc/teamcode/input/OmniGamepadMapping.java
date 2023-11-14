package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;

public class OmniGamepadMapping {
  private Gamepad gamepad;

  public OmniGamepadMapping(Gamepad gamepad) {
    this.gamepad = gamepad;
  }

  public GamepadInputInfo generateInput() {
    return new GamepadInputInfo(-gamepad.left_stick_y,
            -gamepad.left_stick_x,
            -gamepad.right_stick_x);
  }
}
