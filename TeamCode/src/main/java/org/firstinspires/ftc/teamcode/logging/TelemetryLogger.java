package org.firstinspires.ftc.teamcode.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/**
 * Logs messages to the driver station through a {@link Telemetry} object.
 * <p>
 * Messages are persistent only until the current OpMode is stopped.
 */
public class TelemetryLogger implements RobotLogger {
  private final ArrayList<String> msgs;
  private boolean wasAutoClear;
  private boolean flush;
  private Telemetry telemetry;

  public TelemetryLogger(Telemetry telemetry) {
    this.telemetry = telemetry;
    msgs = new ArrayList<>();
  }

  /**
   * Controls whether logged messages will be immediately written to the driver station or cached
   * until the next call to {@link #addTelemetry()}.
   * @param flushMode If true, logged messages will immediately appear on the driver station.
   */
  public void setFlushMode(boolean flushMode) {
    if (flushMode == flush) {
      throw new IllegalStateException("Redundant change to flush mode.");
    }
    if (flushMode) {
      wasAutoClear = telemetry.isAutoClear();
      telemetry.setAutoClear(false);
    } else if (!flushMode) {
      telemetry.setAutoClear(wasAutoClear);
    }
    flush = flushMode;
  }

  /**
   * Writes each logged message to the driver station.
   * <p>
   * Call before {@link Telemetry#update()}.
   */
  public void addTelemetry() {
    for (String msg : msgs) {
      telemetry.addLine(msg);
    }
  }

  public void log(String str) {
    msgs.add(str);
    if (flush) {
      telemetry.update();
    }
  }
}
