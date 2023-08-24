package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.AutoDriveTurnCommandGroup;
import org.firstinspires.ftc.teamcode.Lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Drive to Distance", group = "Auto OpMode")
public class AutoDriveTurnOpMode extends CommandOpMode {
    Hw hw;
    DriveSubsystem drive;

    @Override
    public void initialize() {
        hw = new Hw(this);
        hw.init();

        // Create Subsystems
        drive = new DriveSubsystem(this);

        // Create Commands
        AutoDriveTurnCommandGroup auto = new AutoDriveTurnCommandGroup(this, drive);

        // Register subsystems
        register(drive);

        // Schedule the auto play to run
        CommandScheduler.getInstance().schedule(auto);
    }
    @Override
    public void runOpMode() throws InterruptedException{
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetry.update();
        }
        reset();
    }
}
