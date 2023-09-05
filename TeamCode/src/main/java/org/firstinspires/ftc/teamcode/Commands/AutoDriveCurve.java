package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.DAngle;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

/**
 *  Option 1. Paths
 *    Create a table of distance, robot angle travel angle.
 *    Limitations: travel angle can only be 0,60x
 *    Fix: Pose estimator no PID velocity
 */
public class AutoDriveCurve extends CommandBase {
    CommandOpMode m_opMode;
    DriveSubsystem m_drive;
    DAngle m_angle;
    double m_inches;
    double m_timeOut;
    double m_maxSpeed;
    ProfiledPIDController drivePID = new ProfiledPIDController(0.1,0.01,0, new TrapezoidProfile.Constraints(200,400));
    PIDController rotPID = new PIDController(.01,0,0);
    ElapsedTime m_elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    ElapsedTime m_pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public AutoDriveCurve(CommandOpMode _opMode, DriveSubsystem _drive, double _inches, double _maxSpeed, DAngle _angle, double _timeOut) {
        m_opMode = _opMode;
        m_drive = _drive;
        m_angle = _angle;
        m_inches = _inches;
        m_timeOut = _timeOut;
        m_maxSpeed = _maxSpeed;

    }
    @Override
    public void initialize(){
        drivePID.reset();
        drivePID.setTolerance(0.2,1);

        m_drive.resetMotors();
    }
    @Override
    public void execute(){
        double pid = -drivePID.calculate(m_drive.getDriveDistanceInches(m_angle), m_inches);
        double rot = -rotPID.calculate(m_drive.getRobotAngle(), 0);
        // FIXME: The angle alone gives a normalized value which means speed will be max
        double y = pid * Math.sin(Math.toRadians(DAngle.getAngle(m_angle)));
        double x = pid * Math.cos(Math.toRadians(DAngle.getAngle(m_angle)));
        m_drive.driveXY(y,x,rot);
        while(m_pidTimer.seconds() < 0.05){}
        m_pidTimer.reset();

    }
    @Override
    public boolean isFinished(){
        if(m_elapsedTimer.seconds() > m_timeOut || drivePID.atGoal()){
            return true;
        }
        return false;
    }
}
