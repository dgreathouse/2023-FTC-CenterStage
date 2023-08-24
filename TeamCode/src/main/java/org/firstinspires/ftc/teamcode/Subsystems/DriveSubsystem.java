package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Lib.DAngle;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.MyMath;
import org.firstinspires.ftc.teamcode.Lib.k;

public class DriveSubsystem extends SubsystemBase {
    // Declare the MotorEx and Vector2D classes for each motor
    private MotorEx m_flDrive, m_frDrive, m_bDrive;
    private Vector2d m_flVector, m_frVector, m_bVector;
    // Declare a CommandOpMode variable
    private CommandOpMode m_opMode;
    private boolean m_isFieldOriented = true;

    /** Class Constructor
     *
     * @param _opMode The opMode used which will be teleOp or Autonomous
     */
    public DriveSubsystem(CommandOpMode _opMode) {
        m_opMode = _opMode;

        m_flVector = new Vector2d(Math.cos(30.0 * (Math.PI / 180.0)), Math.sin(30.0 * (Math.PI / 180.0)));
        m_frVector = new Vector2d(Math.cos(150.0 * (Math.PI / 180.0)), Math.sin(150.0 * (Math.PI / 180.0)));
        m_bVector = new Vector2d(Math.cos(270.0 * (Math.PI / 180.0)),  Math.sin(270.0 * (Math.PI / 180.0)));

        initHardware();

    }
    public void initHardware(){
        m_flDrive = new MotorEx(m_opMode.hardwareMap, Hw.s_fl, Motor.GoBILDA.RPM_435);
        m_flDrive.setInverted(false);
        m_flDrive.setRunMode(Motor.RunMode.RawPower);
        m_flDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_flDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        m_flDrive.encoder.setDirection(Motor.Direction.REVERSE);

        m_frDrive = new MotorEx(m_opMode.hardwareMap, Hw.s_fr, Motor.GoBILDA.RPM_435);
        m_frDrive.setInverted(false);
        m_frDrive.setRunMode(Motor.RunMode.RawPower);
        m_frDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_frDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        m_frDrive.encoder.setDirection(Motor.Direction.REVERSE);

        m_bDrive = new MotorEx(m_opMode.hardwareMap, Hw.s_b, Motor.GoBILDA.RPM_435);
        m_bDrive.setInverted(false);
        m_bDrive.setRunMode(Motor.RunMode.RawPower);
        m_bDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_bDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        m_bDrive.encoder.setDirection(Motor.Direction.REVERSE);
    }

    /**
     *
     * @param _ySpeed The forward speed in +/- 1 + is forward
     * @param _xSpeed The strafe speed in +/- 1 left is positive
     * @param _zRotation The rotation speed in +/- 1 CCW is positive
     * @param _gyroAngle The angle of the robot for use with field oriented mode.
     */
    public void driveCartesianIK(double _ySpeed, double _xSpeed, double _zRotation, double _gyroAngle) {
        // Create a new vector2D from the X and Y inputs that contains the angle and magnitude
        Vector2d input = new Vector2d(_ySpeed, _xSpeed);
        // create a variable that holds the new vector if we are using the gyro for field oriented mode
        Vector2d fieldOrientedVector = m_isFieldOriented ? input.rotateBy(_gyroAngle) : input.rotateBy(0);
        fieldOrientedVector.normalize();
        // Calculate the individual motor speeds based on the desired vector and the wheel orientation.
        // TODO: put a PID around the rotation so the robot stays at one angle while driving.
        double flSpeed = fieldOrientedVector.scalarProject(m_flVector) + _zRotation;
        double frSpeed = fieldOrientedVector.scalarProject(m_frVector) + _zRotation;
        double bSpeed = fieldOrientedVector.scalarProject(m_bVector) + _zRotation;

        // Set the motor Speeds
        // TODO: change the motor to be in velocity mode for better control. Set RunMode and scale.
        m_flDrive.set(flSpeed);
        m_frDrive.set(frSpeed);
        m_bDrive.set(bSpeed);

    }
    public void toggleIsFieldOriented(){
        m_isFieldOriented = !m_isFieldOriented;
    }
    /**
     *
     * @return The robot angle in degrees with CCW as positive
     */
    public double getRobotAngle(){
        YawPitchRollAngles angles = Hw.s_imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset the gyro Yaw which is what is used for the robot angle
     */
    public void resetGyro(){
        Hw.s_imu.resetYaw();
    }

    /**
     * Reset the motor encoders back to zero
     */
    public void resetMotors(){
        m_flDrive.resetEncoder();
        m_bDrive.resetEncoder();
        m_frDrive.resetEncoder();

    }

    /**
     *
     * @param _angle The angle we can move in as defined in the DAngle Enum
     * @return The distance traveled in inches.
     */
    public double getDriveDistanceInches(DAngle _angle){
        double rtn = 0;
        double left = m_flDrive.getDistance();
        double right = m_frDrive.getDistance();
        double back = m_bDrive.getDistance();
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
        m_opMode.telemetry.addData("L Inches = ", m_flDrive.getDistance());
        m_opMode.telemetry.addData("R Inches = ", m_frDrive.getDistance());
        m_opMode.telemetry.addData("B Inches = ", m_bDrive.getDistance());
        m_opMode.telemetry.addData("L Vel = ", m_flDrive.getVelocity());
        m_opMode.telemetry.addData("R Vel = ", m_frDrive.getVelocity());
        m_opMode.telemetry.addData("B Vel = ", m_bDrive.getVelocity());
        m_opMode.telemetry.addData("Robot Angle = ",getRobotAngle());
        m_opMode.telemetry.addData("DriveDistance = ", getDriveDistanceInches(DAngle.ang_0));

    }
}
