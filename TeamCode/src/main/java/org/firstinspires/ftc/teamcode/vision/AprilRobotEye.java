package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilRobotEye implements RobotEye {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    public AprilRobotEye(CameraName cameraName) {
        List<VisionProcessor> processors = new ArrayList<>();
        tagProcessor = buildTagProcessor();
        processors.add(tagProcessor);
        visionPortal = buildVisionPortal(cameraName, processors);
        visionPortal.resumeStreaming();
    }
    private static AprilTagProcessor buildTagProcessor() {
        return new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
    }
    private static VisionPortal buildVisionPortal(CameraName cameraName, List<VisionProcessor> processors) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        processors.forEach(builder::addProcessor);
        return builder.enableLiveView(true)
                .setCamera(cameraName)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
    }
    @Override
    public List<AprilTagDetection> getTagDetections() {
        return tagProcessor.getDetections();
    }
}
