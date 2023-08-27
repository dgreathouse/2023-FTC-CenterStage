package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    ElapsedTime m_elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double m_timerAvg = 0.0;
    double m_timerCnt = 0;
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
        // register(drive);

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
            // Calculate the run rate of this loop
            m_timerAvg += m_elapsedTimer.seconds();
            if(m_timerCnt++ >= 10){
                telemetry.addData("RunLoopRate = ", m_timerAvg/10);
                m_timerAvg = 0.0; m_timerCnt = 0;
            }
            telemetry.update();
            // wait till timer is > 50ms to try an create a stable run rate
            while(m_elapsedTimer.seconds() < 0.05){} m_elapsedTimer.reset();
        }
        reset();
    }
}
