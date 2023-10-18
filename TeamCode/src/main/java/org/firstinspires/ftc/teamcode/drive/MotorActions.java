package org.firstinspires.ftc.teamcode.drive;

import java.util.HashSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

// Independent of robot configuration.
public class MotorActions {
  private Set<String> motorNames;
  private Map<String, Double> initialEncoders;
  private Map<String, Double> speeds;
  private Map<String, Double> finalEncoders;
  private Map<String, Double> encoders;
  private Point3d goalPosition;
  private double goalRotation;

  private MotorActions(Map<String, Double> initialEncoders, Map<String, Double> speeds,
    Map<String, Double> finalEncoders) {
    this.initialEncoders = initialEncoders;
    this.speeds = speeds;
    this.finalEncoders = finalEncoders;
  }
  
  public double getInitialEncoder(String motorName) {
    checkMotorName(motorName);
    return initialEncoders.get(motorName);
  }
  public double getSpeed(String motorName) {
    checkMotorName(motorName);
    return speeds.get(motorName);
  }
  public double getFinalEncoder(String motorName) {
    checkMotorName(motorName);
    return finalEncoders.get(motorName);
  }
  public void setFinalEncoder(String motorName, double value) {
    checkMotorName(motorName);
    finalEncoders.put(motorName, value);
  }
  public double getEncoder(String motorName) {
    checkMotorName(motorName);
    return encoders.get(motorName);
  }
  public void setEncoder(String motorName) {
    checkMotorName(motorName);
    encoders.put(motorName);
  }
  private void checkMotorName(String motorName) {
    if (!motorNames.contains(motorName))
      throw IllegalArgumentException("Motor " + motorName + " invalid.");
    }
  }

  public static class Builder {
    private Set<String> motorNames;
    private Map<String, Double> initialEncoders;
    private Map<String, Double> speeds;
    private Map<String, Double> finalEncoders;
    private boolean finalized;

    public Builder() {
      motorNames = new HashSet<String>();
      initialEncoders = new HashMap<Double, String>();
      speeds = new HashMap<Double, String>();
      finalEncoders = new HashMap<Double, String>();
    }

    public void setInitialEncoder(String motorName, double value) {
      checkFinalized();
      motorNames.add(motorName);
      initialEncoders.put(motorName, value);
    }
    public void setSpeed(String motorName, double value) {
      checkFinalized();
      motorNames.add(motorName);
      speeds.put(motorName, value);
    }
    public void setFinalEncoder(String motorName, double value) {
      checkFinalized();
      motorNames.add(motorName);
      initialEncoders.put(motorName, value);
    }
    public MotorActions build() {
      checkFinalized();
      finalized = true;
      return new MotorActions(motorNames, initialEncoders, speeds, finalEncoders);
    }
    private void checkFinalized() {
      if (finalized) {
        throw IllegalStateException("Builder already finalized.");
      }
    }
  }
}
