package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public interface RobotEye {
    List<AprilTagDetection> getTagDetections();
}
