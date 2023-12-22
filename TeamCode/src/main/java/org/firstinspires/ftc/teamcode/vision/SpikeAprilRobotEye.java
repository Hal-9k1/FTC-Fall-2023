package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

public class SpikeAprilRobotEye implements RobotEye {
    private AprilTagProcessor tagProcessor;
    private TfodProcessor spikeProcessor;
    private VisionPortal visionPortal;
    private double spikeWidthMeters;
    private double cameraFov;
    public SpikeAprilRobotEye(CameraName cameraName, double spikeWidthMeters, double cameraFov) {
        List<VisionProcessor> processors = new ArrayList<>();
        tagProcessor = buildTagProcessor();
        processors.add(tagProcessor);
        spikeProcessor = buildSpikeProcessor();
        processors.add(spikeProcessor);
        visionPortal = buildVisionPortal(cameraName, processors);
        visionPortal.resumeStreaming();
        this.spikeWidthMeters = spikeWidthMeters;
        this.cameraFov = cameraFov;
    }
    private static AprilTagProcessor buildTagProcessor() {
        return new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
    }
    private static TfodProcessor buildSpikeProcessor() {
        return new TfodProcessor.Builder()
                .build(); // use default model, white pixel
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
    @Override
    public List<Matrix4d> getSpikeDetections() {
        return spikeProcessor.getRecognitions().stream().map(recognition -> {
          // conversion factor valid only in the plane perpendicular to the camera ray containing
          // the spike:
          double distPerPx = spikeWidthMeters / recognition.getWidth();
          Matrix3d id3 = new Matrix3d();
          id3.setIdentity();
          return new Matrix4d(id3, new Vector3d(
            recognition.getImageWidth() * distPerPx / Math.tan(cameraFov / 2) / 2,
            (recognition.getImageWidth() - (recognition.getLeft() + recognition.getRight()) / 2)
              * distPerPx,
            0), 1.0);
        }).collect(Collectors.toList());
    }
    @Override
    public List<Matrix4d> getPixelDetections() {
        return new ArrayList<>();
    }
}
