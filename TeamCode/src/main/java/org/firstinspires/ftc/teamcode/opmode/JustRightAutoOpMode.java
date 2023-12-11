package org.firstinspires.ftc.teamcode.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.logging.TelemetryLogger;

import javax.vecmath.Vector2d;

/**
 * Just moves to the right.
 * That's it. No navigator or pilot, or anything remotely interesting whatsoever.
 */
@Autonomous(name="Just Right Auto", group="Iterative OpMode")
public class JustRightAutoOpMode extends OpMode {
    private DriveSystem driveSystem;
    private ElapsedTime runtime;
    private TelemetryLogger logger;

    @Override
    public void init() {
        logger = new TelemetryLogger(telemetry);
        logger.setFlushMode(true);
        driveSystem = new MecanumDriveSystem(hardwareMap);
        logger.setFlushMode(false);
        telemetry.update();
        telemetry.addData("Status", "Initialized");
        logger.addTelemetry();
        telemetry.update();
    }
    @Override
    public void start() {
        runtime = new ElapsedTime();
    }
    @Override
    public void loop() {
        if (runtime.milliseconds() < 7000) {
            driveSystem.move(new Vector2d(-1.0, 0.0), 0.25);
            //driveSystem.turn(1.0, 1.0);
            driveSystem.exec();
            telemetry.addData("Status", "Running");
        } else {
            driveSystem.halt();
            telemetry.addData("Status", "Finished");
        }
        telemetry.addData("Runtime", runtime.toString());
        logger.addTelemetry();
        telemetry.update();
    }
}
