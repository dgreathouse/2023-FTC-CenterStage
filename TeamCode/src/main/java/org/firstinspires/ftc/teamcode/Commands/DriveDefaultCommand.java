package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;


import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.MyMath;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class DriveDefaultCommand extends CommandBase {
    // Declare a variable called "drive" of type "DriveSubsystem"
    DriveSubsystem m_drive;
    // Declare a variable called "opMode" of type "CommandOpMode"
    CommandOpMode m_opMode;
    // Create local variables of type double to store the stick X,Y,Z values and Angle of robot.
    double m_x, m_y, m_z, m_ang;

    /** Constructor of class
     *
     * @param _opMode The opMode used which will be teleOp or Autonomous
     * @param _drive The DriveSubsystem instance variable
     */
    public DriveDefaultCommand(CommandOpMode _opMode, DriveSubsystem _drive) {

        m_drive = _drive;    // Set the local "drive" variable to the parameter "_drive"
        m_opMode = _opMode;  // Set the local "opMode" variable to the parameter "_opMode"
        // Set the requirements for the Command. This always must be done for a "Command"
        // The requirement is any subsystem that will be used by this command.
        addRequirements(m_drive);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        // Get the X,Y, and Z axis values from the Driver joystick.
        // The values from the joystick are always in the range of +/- 1.0
        // Deadband is a applied to prevent the small on center offset from making the robot move when the thumb is off the stick
        m_y = MyMath.applyDeadband(-Hw.s_gpDriver.getLeftY(), .05, 1.0);
        m_x = MyMath.applyDeadband(Hw.s_gpDriver.getLeftX(), .05, 1.0);
        m_z = MyMath.applyDeadband(-Hw.s_gpDriver.getRightX(), .05, 1.0);

        // Scale the turning rotation down by 1/2
        m_z = m_z * 0.5;
        // Call the drive method "driveCaresianXY" with the stick X,Y,Z and angle parameters
        m_drive.driveCartesianXY(m_y,-m_x, m_z);

    }
    @Override
    public void end(boolean _interrupted){
        m_drive.driveCartesianXY(0,0,0);
    }

}
