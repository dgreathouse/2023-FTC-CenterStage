package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;


import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.MyMath;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class DriveDefaultCommand extends CommandBase {
    // Declare a variable called "drive" of type "DriveSubsystem"
    DriveSubsystem drive;
    // Declare a variable called "opMode" of type "CommandOpMode"
    CommandOpMode opMode;
    // Create local variables of type double to store the stick X,Y,Z values and Angle of robot.
    double x,y,z, ang;

    /** Constructor of class
     *
     * @param _opMode The opMode used which will be teleOp or Autonomous
     * @param _drive The DriveSubsystem instance variable
     */
    public DriveDefaultCommand(CommandOpMode _opMode, DriveSubsystem _drive) {

        drive = _drive;    // Set the local "drive" variable to the parameter "_drive"
        opMode = _opMode;  // Set the local "opMode" variable to the parameter "_opMode"
        // Set the requirements for the Command. This always must be done for a "Command"
        // The requirement is any subsystem that will be used by this command.
        addRequirements(drive);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        // Get the X,Y, and Z axis values from the Driver joystick.
        // The values from the joystick are always in the range of +/- 1.0
        // Deadband is a applied to prevent the small on center offset from making the robot move when the thumb is off the stick
        y = MyMath.applyDeadband(-Hw.s_gpDriver.getLeftY(), .05, 1.0);
        x = MyMath.applyDeadband(Hw.s_gpDriver.getLeftX(), .05, 1.0);
        z = MyMath.applyDeadband(-Hw.s_gpDriver.getRightX(), .05, 1.0);
        // Get the angle of the robot in Degrees
        ang = drive.getRobotAngle();
        // Scale the turning rotation down by 1/2
        z = z * 0.5;
        // Call the drive method "driveCaresianIK" with the stick X,Y,Z and angle parameters
        drive.driveCartesianIK(y,-x,z,ang);
        // Display the values of X,Y,Z on the driveStation.
        opMode.telemetry.addData("x = ", -x);
        opMode.telemetry.addData("y = ", y);
        opMode.telemetry.addData("z = ", z);
    }
    @Override
    public void end(boolean _interrupted){
        drive.driveCartesianIK(0,0,0,0);
    }

}
