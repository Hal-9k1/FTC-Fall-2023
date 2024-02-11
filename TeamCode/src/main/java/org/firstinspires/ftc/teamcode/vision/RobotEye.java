package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import javax.vecmath.Matrix4d;

public interface RobotEye {
    List<AprilTagDetection> getTagDetections();
    List<Matrix4d> getSpikeDetections();
    List<Matrix4d> getPixelDetections();
}
