package org.firstinspires.ftc.teamcode.logging;

/**
 * Discards logged messages. Useful for production.
 */
public class NopLogger implements RobotLogger {
  public void log(String msg) { } // do nothing
}
