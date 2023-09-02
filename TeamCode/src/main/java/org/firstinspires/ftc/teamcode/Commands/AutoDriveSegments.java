package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.Segment;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.List;
import java.util.concurrent.TimeUnit;

/** Drive the robot at a speed and angle for a certain time. Also rotate the robot to an angle while driving
   Since the robot drives at a velocity the only changing variable to get a consistent distance is
   the battery which changes the time it takes to go from stop to full velocity. We hope this change
   will not affect the accuracy to much. There is no PID on distance and the coast time will need
   to be considered.
 */
public class AutoDriveSegments extends CommandBase {
    CommandOpMode m_opMode;
    DriveSubsystem m_drive;
    double m_angle;
    double m_robotAngle;
    double m_timeOut;
    double m_speed;
    List<Segment> m_segments;
    Segment m_segment;
    int m_index = 0;
    boolean m_isFinished = false;
    PIDController rotPID = new PIDController(.01,0,0);
    Timing.Timer m_timer;


    public AutoDriveSegments(CommandOpMode _opMode, DriveSubsystem _drive, List<Segment> _segments) {
        m_opMode = _opMode;
        m_drive = _drive;
        m_segments = _segments;
    }

    @Override
    public void initialize(){
        rotPID.reset();
        m_timer = new Timing.Timer(m_segments.get(0).m_time, TimeUnit.MILLISECONDS);
        m_timer.start();

    }
    @Override
    public void execute(){
        m_segment = m_segments.get(m_index);
        //TODO: if start move we may be able to ramp the magnitude up to get a consistent starting distance.
        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_segment.m_robotAngle);
        m_drive.drivePolar(m_segment.m_angle, m_segment.m_magnitude, rot);

        if(m_timer.done()){
            m_index++;
            //m_isFinished = m_index == m_segments.size() ? true : false;
            if(m_index == m_segments.size()){
                m_isFinished = true;
            }
            m_timer = new Timing.Timer(m_segments.get(m_index).m_time, TimeUnit.MILLISECONDS);
            m_timer.start();
        }

    }
    @Override
    public boolean isFinished(){
        //return m_isFinished == true ? true : false;
        if(m_isFinished){
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean _interrupted){
        m_drive.disableMotors();
    }
}
