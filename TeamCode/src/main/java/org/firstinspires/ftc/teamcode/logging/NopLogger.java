package org.firstinspires.ftc.teamcode.logging;

/**
 * Discards logged messages. Useful for production.
 */
public class NopLogger implements RobotLogger {
  @Override
  public void log(String msg) { } // do nothing
}
