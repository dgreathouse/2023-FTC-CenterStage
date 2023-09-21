package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.Interpolate;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoDriveToDistance extends CommandBase
{
    CommandOpMode m_opMode;
    DriveSubsystem m_drive;
    boolean m_isFinished = false;
    Timing.Timer m_timer;
    double m_timeout;
    double m_robotAngle;
    double m_driveAngle;
    double m_speed;
    PIDController rotPID = new PIDController(.01,0,0);
    public AutoDriveToDistance(CommandOpMode _opMode, DriveSubsystem _drive, double[] _distanceX, double[] _timeY, double _distance, double _speed, double _driveAngle, double _robotAngle){
        m_opMode = _opMode;
        m_drive = _drive;
        m_timeout = Interpolate.getY(_distanceX,_timeY,_distance);
        m_robotAngle = _robotAngle;
        m_driveAngle = _driveAngle;
        m_speed = _speed;
    }

    @Override
    public void initialize(){
        rotPID.reset();
        m_timer = new Timing.Timer((int)(m_timeout*1000), TimeUnit.MILLISECONDS);
        m_timer.start();
    }
    @Override
    public void execute(){
        double rot = -rotPID.calculate(m_drive.getRobotAngle(),m_robotAngle);
        m_drive.drivePolar(m_driveAngle,m_speed/100.0, rot);
        if(m_timer.done()){
            m_isFinished = true;
        }
    }
    @Override
    public boolean isFinished(){
        return m_isFinished == true ? true : false;
    }
    @Override
    public void end(boolean _interrupted){
        m_drive.disableMotors();
    }
}
