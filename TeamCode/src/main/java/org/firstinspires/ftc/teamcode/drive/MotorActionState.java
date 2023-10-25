package org.firstinspires.ftc.teamcode.drive;

import java.util.HashSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import javax.vecmath.Matrix4d;

// Independent of robot configuration.
public class MotorActionState {
  private Set<String> motorNames;
  private Map<String, Double> initialEncoders;
  private Map<String, Double> speeds;
  private Map<String, Double> finalEncoders;
  private Map<String, Double> encoders;
  private Matrix4d goalTransform;

  private MotorActionState(Set<String> motorNames, Map<String, Double> initialEncoders,
    Map<String, Double> speeds, Map<String, Double> finalEncoders, Matrix4d goalTransform) {
    this.motorNames = motorNames;
    this.initialEncoders = initialEncoders;
    this.speeds = speeds;
    this.finalEncoders = finalEncoders;
    this.goalTransform = goalTransform;
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
  public void setEncoder(String motorName, double value) {
    checkMotorName(motorName);
    encoders.put(motorName, value);
  }
  /**
   * Gets the goal transform of this set of motor actions relative to the robot's initial transform.
   *
   * @return The goal transform in robot space relative to the pre-action transform.
   */
  public Matrix4d getGoalTransform() {
    return new Matrix4d(goalTransform); // don't expose mutable object
  }

  private void checkMotorName(String motorName) {
    if (!motorNames.contains(motorName)) {
      throw new IllegalArgumentException("Motor " + motorName + " invalid.");
    }
  }

  public static class Builder {
    private Set<String> motorNames;
    private Map<String, Double> initialEncoders;
    private Map<String, Double> speeds;
    private Map<String, Double> finalEncoders;
    private Matrix4d goalTransform;
    private boolean finalized;

    public Builder() {
      motorNames = new HashSet<String>();
      initialEncoders = new HashMap<String, Double>();
      speeds = new HashMap<String, Double>();
      finalEncoders = new HashMap<String, Double>();
      goalTransform = null;
      finalized = false;
    }

    public Builder setInitialEncoder(String motorName, double value) {
      checkFinalized();
      motorNames.add(motorName);
      initialEncoders.put(motorName, value);
      return this;
    }
    public Builder setSpeed(String motorName, double value) {
      checkFinalized();
      motorNames.add(motorName);
      speeds.put(motorName, value);
      return this;
    }
    public Builder setFinalEncoder(String motorName, double value) {
      checkFinalized();
      motorNames.add(motorName);
      initialEncoders.put(motorName, value);
      return this;
    }
    public Builder setGoalTransform(Matrix4d goalTransform) {
      goalTransform = new Matrix4d(goalTransform);
      return this;
    }
    public MotorActionState build() {
      checkFinalized();
      finalized = true;
      if (goalTransform == null) {
        throw new IllegalStateException("Goal transform has not been set.");
      }
      return new MotorActionState(motorNames, initialEncoders, speeds, finalEncoders, goalTransform);
    }
    private void checkFinalized() {
      if (finalized) {
        throw new IllegalStateException("Builder already finalized.");
      }
    }
  }
}
