package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Lib.DAngle;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.MyMath;

public class DriveSubsystem extends SubsystemBase {
    // Declare the MotorEx classes for each motor
    private MotorEx flDrv, frDrv, bDrv;
    public static final Vector2d m_flVec = new Vector2d(Math.cos(30.0 * (Math.PI / 180.0)), Math.sin(30.0 * (Math.PI / 180.0)));
    public static final Vector2d m_frVec = new Vector2d(Math.cos(150.0 * (Math.PI / 180.0)), Math.sin(150.0 * (Math.PI / 180.0)));
    public static final Vector2d m_bVec = new Vector2d(Math.cos(270.0 * (Math.PI / 180.0)),  Math.sin(270.0 * (Math.PI / 180.0)));

    CommandOpMode m_opMode;
    double m_ySpeed, m_xSpeed, m_zRotation;

    /** Class Constructor
     *
     * @param _opMode The opMode used which will be teleOp or Autonomous
     * @param _hw The Hardware reference that contains all the hardware instances.
     */
    public DriveSubsystem(CommandOpMode _opMode, Hw _hw) {
        m_opMode = _opMode;
        // Define the motor reference to be the MotorEx defined in Hw.java class.
        flDrv = _hw.flDrive;
        frDrv = _hw.frDrive;
        bDrv = _hw.bDrive;
    }
    public void driveCartesianIK(double _ySpeed, double _xSpeed, double _zRotation, double _gyroAngle) {
        double flSpeed = 0.0;
        double frSpeed = 0.0;
        double bSpeed = 0.0;

        MyMath.clamp(_ySpeed, -1.0, 1.0);
        MyMath.clamp(_xSpeed, -1.0, 1.0);
        MyMath.clamp(_zRotation, -1.0, 1.0);

        m_zRotation = _zRotation;

        Vector2d input = new Vector2d(_ySpeed, _xSpeed);
        Vector2d fieldOriented = input.rotateBy(_gyroAngle);

        flSpeed = fieldOriented.scalarProject(m_flVec) + _zRotation;
        frSpeed = fieldOriented.scalarProject(m_frVec) + _zRotation;
        bSpeed = fieldOriented.scalarProject(m_bVec) + _zRotation;

        flDrv.set(flSpeed);
        frDrv.set(frSpeed);
        bDrv.set(bSpeed);

    }
    public double getRobotAngle(){
        YawPitchRollAngles angles = Hw.imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
    public void resetGyro(){
        Hw.imu.resetYaw();
    }
    public void resetMotors(){
        flDrv.resetEncoder();
        bDrv.resetEncoder();
        frDrv.resetEncoder();

    }
    public double getDriveDistanceInches(DAngle _angle){
        double rtn = 0;
        double left = flDrv.getDistance();
        double right = frDrv.getDistance();
        double back = bDrv.getDistance();
        switch (_angle){
            case ang_0: // Left, Right
                rtn = (left - right)/2;
                break;
            case ang_180:
                rtn = -(left - right)/2;
                break;
            case ang_60: // Left, Back
                rtn = (left - back)/2;
                break;
            case ang_240:
                rtn = -(left - back)/2;
                break;
            case ang_120: // Right, Back
                rtn = -(-right + back)/2;
                break;
            case ang_300:
                rtn = (-right + back)/2;
                break;
            default:
                rtn = 0;
                break;
        }
        return rtn;
    }
    @Override
    public void periodic(){
        m_opMode.telemetry.addData("L Inches = ", flDrv.getDistance());
        m_opMode.telemetry.addData("R Inches = ", frDrv.getDistance());
        m_opMode.telemetry.addData("B Inches = ", bDrv.getDistance());
        m_opMode.telemetry.addData("L Vel = ", flDrv.getVelocity());
        m_opMode.telemetry.addData("R Vel = ", frDrv.getVelocity());
        m_opMode.telemetry.addData("B Vel = ", bDrv.getVelocity());
        m_opMode.telemetry.addData("Robot Angle = ",getRobotAngle());
        m_opMode.telemetry.addData("DriveDistance = ", getDriveDistanceInches(DAngle.ang_0));

    }
}
