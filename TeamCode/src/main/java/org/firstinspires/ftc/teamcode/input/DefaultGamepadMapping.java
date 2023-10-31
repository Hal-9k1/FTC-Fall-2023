package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DefaultGamepadMapping {
  private Gamepad gamepad;

  public DefaultGamepadMapping(Gamepad gamepad) {
    this.gamepad = gamepad;
  }

  public GamepadInputInfo generateInput() {
    return new GamepadInputInfo(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
  }
}
