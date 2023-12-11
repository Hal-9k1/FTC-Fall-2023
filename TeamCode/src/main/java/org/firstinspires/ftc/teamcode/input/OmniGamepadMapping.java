package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Generates GamepadInputInfo from a gamepad.
 * <p>
 * The left joystick controls holonomic motion, horizontal movement of the right joystick turns the
 * robot (left turns the robot counterclockwise), and the right bumper signals the plane launcher.
 */
public class OmniGamepadMapping {
  private Gamepad gamepad;
  private GamepadInputInfo inputInfo;
  public OmniGamepadMapping(Gamepad gamepad) {
    this.gamepad = gamepad;
  }

  /**
   * Generates input info from the current state of the input devices.
   * @see #getInput()
   */
  public void generateInput() {
    inputInfo = new GamepadInputInfo(-gamepad.left_stick_y,
            -gamepad.left_stick_x,
            -gamepad.right_stick_x,
            gamepad.right_bumper);
  }

  /**
   * Gets the most recently generated input info by {@link #generateInput()}.
   * <p>
   * Successive calls before another call to {@code generateInput()} are guaranteed to return the
   * same object.
   * @return The generated input info.
   */
  public GamepadInputInfo getInput() {
    return inputInfo;
  }
}
