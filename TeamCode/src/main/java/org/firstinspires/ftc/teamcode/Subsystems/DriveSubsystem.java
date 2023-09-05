package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Lib.DAngle;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.KiwiDrive;
import org.firstinspires.ftc.teamcode.Lib.k;

public class DriveSubsystem extends SubsystemBase {
    // Declare the MotorEx and Vector2D classes for each motor
    private MotorEx m_lDrive, m_rDrive, m_bDrive;
    private KiwiDrive m_drive;


    // Declare a CommandOpMode variable
    private CommandOpMode m_opMode;

    /** Class Constructor
     *
     * @param _opMode The opMode used which will be teleOp or Autonomous
     */
    public DriveSubsystem(CommandOpMode _opMode) {
        m_opMode = _opMode;

        initHardware();

        m_drive = new KiwiDrive(m_lDrive,m_rDrive, m_bDrive);

    }
    public void initHardware(){
        m_lDrive = new MotorEx(m_opMode.hardwareMap, Hw.s_fl, Motor.GoBILDA.RPM_435);
        m_lDrive.setInverted(true);
        m_lDrive.setRunMode(Motor.RunMode.VelocityControl);
        m_lDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_lDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        m_lDrive.encoder.setDirection(Motor.Direction.FORWARD);
        m_lDrive.setVeloCoefficients(1.0,0.01,0);

        m_rDrive = new MotorEx(m_opMode.hardwareMap, Hw.s_fr, Motor.GoBILDA.RPM_435);
        m_rDrive.setInverted(true);
        m_rDrive.setRunMode(Motor.RunMode.VelocityControl);
        m_rDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_rDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        m_rDrive.encoder.setDirection(Motor.Direction.REVERSE);
        m_rDrive.setVeloCoefficients(1.0,0.01,0);

        m_bDrive = new MotorEx(m_opMode.hardwareMap, Hw.s_b, Motor.GoBILDA.RPM_435);
        m_bDrive.setInverted(true);
        m_bDrive.setRunMode(Motor.RunMode.VelocityControl);
        m_bDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_bDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        m_bDrive.encoder.setDirection(Motor.Direction.REVERSE);
        m_bDrive.setVeloCoefficients(1.0,0.01,0);

    }

    /**
     *
     * @param _strafeSpeed The forward speed in +/- 1 left is positive
     * @param _forwardSpeed The strafe speed in +/- 1 forward is positive
     * @param _zRotation The rotation speed in +/- 1 CCW/left is positive
     */
    public void driveXY(double _strafeSpeed, double _forwardSpeed, double _zRotation) {

        m_drive.driveXY(_strafeSpeed,_forwardSpeed,_zRotation,getRobotAngle());

    }

    public void drivePolar(double _angle, double _speed, double _rot){
        m_drive.drivePolar(_angle,_speed,_rot,getRobotAngle());
    }
    public void toggleIsFieldOriented(){
        m_drive.toggleIsFieldOriented();
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
        m_lDrive.resetEncoder();
        m_bDrive.resetEncoder();
        m_rDrive.resetEncoder();

    }

    /**
     *
     * @param _angle The angle we can move in as defined in the DAngle Enum
     * @return The distance traveled in inches.
     */
    public double getDriveDistanceInches(DAngle _angle){
        double rtn = 0;
        double left = m_lDrive.getDistance();
        double right = m_rDrive.getDistance();
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
    public void disableMotors(){
        m_lDrive.set(0);
        m_rDrive.set(0);
        m_bDrive.set(0);
    }
    public void setIsFieldOriented(boolean _val){
        m_drive.setIsFieldOriented(_val);
    }
    @Override
    public void periodic(){
        m_opMode.telemetry.addData("L Inches", m_lDrive.getDistance());
        m_opMode.telemetry.addData("R Inches", m_rDrive.getDistance());
        m_opMode.telemetry.addData("B Inches", m_bDrive.getDistance());
        m_opMode.telemetry.addData("L Vel", m_lDrive.getVelocity());
        m_opMode.telemetry.addData("R Vel", m_rDrive.getVelocity());
        m_opMode.telemetry.addData("B Vel", m_bDrive.getVelocity());
        m_opMode.telemetry.addData("Robot Angle",getRobotAngle());
        m_opMode.telemetry.addData("DriveDistance", getDriveDistanceInches(DAngle.ang_0));
        m_opMode.telemetry.addData("DriveAngle", m_drive.getDriveAngle());
        m_opMode.telemetry.addData("Strafe", m_drive.getStrafe());
        m_opMode.telemetry.addData("Forward",m_drive.getForward());
        m_opMode.telemetry.addData("FieldOriented", m_drive.getIsFieldOriented());
    }
}
