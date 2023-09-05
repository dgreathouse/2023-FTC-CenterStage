package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class KiwiDrive extends RobotDrive{
    MotorEx m_left, m_right, m_back;
    double[] m_speeds = new double[3];
    boolean m_isFieldOriented = true;
    double m_driveAngle = 0;
    double m_strafe, m_forward;
    Vector2d m_vector;
    private Vector2d m_lVector, m_rVector, m_bVector;


    public KiwiDrive(MotorEx _left, MotorEx _right, MotorEx _back){
        m_left = _left; m_right = _right; m_back = _back;
        m_lVector = new Vector2d(Math.cos(Math.toRadians(30.0)), Math.sin(Math.toRadians(30.0)));
        m_rVector = new Vector2d(Math.cos(Math.toRadians(150.0)), Math.sin(Math.toRadians(150.0)));
        m_bVector = new Vector2d(Math.cos(Math.toRadians(270.0)),  Math.sin(Math.toRadians(270.0)));

    }
    public void driveXY(double _strafe, double _forward, double _rotate, double _heading){
        // Set the class values before clipping
        m_strafe = _strafe;
        m_forward = _forward;
        // Limit/Clip the values and scale if needed
        _strafe = clipRange(_strafe);
        _forward = clipRange(_forward);
        _rotate = clipRange(_rotate);
        // Set m_vector of the x,y/forward,strafe values. Notice forward is the Vector2D X value.
        m_vector = m_isFieldOriented ? new Vector2d(_forward,_strafe).rotateBy(_heading) : new Vector2d(_forward,_strafe);
        m_driveAngle = Math.toDegrees(m_vector.angle());  // Set the class driveAngle that was calculated from the m_vector.
        // Scale the individual motor speeds based off the angle of the motor. Then assign to an array for normalize
        m_speeds[0] = m_vector.scalarProject(m_lVector);
        m_speeds[1] = m_vector.scalarProject(m_rVector);
        m_speeds[2] = m_vector.scalarProject(m_bVector);
        // Normalize the speeds which means to scale them by the maximum
        normalize(m_speeds);
        // Set the speeds of the actual motors with the rotation value.
        m_left.set(m_speeds[0] + _rotate);
        m_right.set(m_speeds[1] + _rotate);
        m_back.set(m_speeds[2] + _rotate);
    }
    public void drivePolar(double _angle, double _speed, double _rot, double _heading){
        double x = Math.sin(Math.toRadians(_angle)) * _speed;
        double y = Math.cos(Math.toRadians(_angle)) * _speed;

        driveXY(x,y,_rot,_heading);
    }

    public double[] getSpeeds(){
        return m_speeds;
    }
    @Override
    public void stop() {
        m_left.stopMotor();
        m_right.stopMotor();
        m_back.stopMotor();
    }
    public void setIsFieldOriented(boolean _isFieldOriented){
        m_isFieldOriented = _isFieldOriented;
    }
    public boolean getIsFieldOriented(){
        return m_isFieldOriented;
    }
    public void toggleIsFieldOriented(){
        m_isFieldOriented = !m_isFieldOriented;
    }
    public double getDriveAngle(){return m_driveAngle;}
    public double getStrafe(){return m_strafe;}
    public double getForward(){return m_forward;}
}
