package org.firstinspires.ftc.teamcode.Lib;

public class Segment {
    public final double m_angle;
    public final double m_magnitude;
    public final double m_robotAngle;
    public final long m_time;

    public Segment(double _angle, double _magnitude, double _robotAngle, long _timeMS){
        m_angle = _angle;
        m_magnitude = _magnitude;
        m_robotAngle = _robotAngle;
        m_time = _timeMS;
    }
}
