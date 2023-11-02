package org.firstinspires.ftc.teamcode.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TelemetryLogger implements RobotLogger {
  private final ArrayList<String> msgs;
  private boolean wasAutoClear;
  private boolean flush;
  private Telemetry telemetry;

  public TelemetryLogger(Telemetry telemetry) {
    this.telemetry = telemetry;
    msgs = new ArrayList<>();
  }
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
