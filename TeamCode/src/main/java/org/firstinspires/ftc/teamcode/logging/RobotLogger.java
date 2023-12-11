package org.firstinspires.ftc.teamcode.logging;

/**
 * Provides an opaque mechanism for various parts of the system to log messages for human review.
 */
public interface RobotLogger {
  /**
   * Logs a message.
   * @param msg The message to be logged.
   */
  void log(String msg);
}
