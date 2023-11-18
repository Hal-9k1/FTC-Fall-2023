package org.firstinspires.ftc.teamcode.opmode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;
import org.firstinspires.ftc.teamcode.navigator.BeelineNavigator;
import org.firstinspires.ftc.teamcode.navigator.RobotNavigator;
import org.firstinspires.ftc.teamcode.path.BlindPathPlanner;
import org.firstinspires.ftc.teamcode.path.PathPlanner;
import org.firstinspires.ftc.teamcode.pilot.RobotPilot;
import org.firstinspires.ftc.teamcode.pilot.SimplePilot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

import javax.vecmath.Matrix4d;

@Autonomous
public class SightedPathingAutoOpMode extends OpMode {

    private static final String WEBCAM_NAME = "WEBCAM NAME HERE";
    private TelemetryLogger logger;
    private DriveSystem driveSystem;
    private RobotPilot pilot;
    private RobotNavigator navigator;
    private PathPlanner pathPlanner;
    private ElapsedTime runtime;
    private Matrix4d cameraTransformRS;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        logger = new TelemetryLogger(telemetry);
        logger.setFlushMode(true);
        driveSystem = new MecanumDriveSystem(hardwareMap);
        Matrix4d initialRobotTransform = new Matrix4d();
        initialRobotTransform.setIdentity();
        pilot = new SimplePilot(driveSystem, initialRobotTransform,
                AprilTagGameDatabase.getCenterStageTagLibrary());
        navigator = new BeelineNavigator(pilot);
        pathPlanner = new BlindPathPlanner(navigator);
        tagProcessor = buildTagProcessor();
        visionPortal = buildVisionPortal(hardwareMap.get(WebcamName.class, WEBCAM_NAME),
                Arrays.asList(tagProcessor));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        logger.setFlushMode(false);
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
    public void start() {
        runtime = new ElapsedTime();
        cameraTransformRS = new Matrix4d();
        cameraTransformRS.setIdentity();
    }
    @Override
    public void loop() {
        if (pathPlanner.tick()) {
            telemetry.addData("Status", "Finished");
        } else {
            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", runtime.toString());
        }
        pilot.updateWithTags(cameraTransformRS, tagProcessor.getDetections());
        pilot.addTelemetry(telemetry);
        logger.addTelemetry();
        telemetry.update();
    }
}