package org.firstinspires.ftc.teamcode.pilot;

import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.logging.RobotLogger;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import javax.vecmath.Matrix4d;

public class SimplePilot extends BasePilot {
    public SimplePilot(RobotLogger logger, DriveSystem driveSystem, Matrix4d allianceOriginTransform,
                       Matrix4d initialRobotTransform, AprilTagLibrary tagLibrary) {
        super(logger, driveSystem, allianceOriginTransform, initialRobotTransform, tagLibrary,
                false);
    }
}
