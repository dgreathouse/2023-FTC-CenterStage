package org.firstinspires.ftc.teamcode.Commands;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;


import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.MyMath;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class DriveDefaultCommand extends CommandBase {

    DriveSubsystem drive;
    CommandOpMode opMode;

    double x,y,z, ang;
    public DriveDefaultCommand(CommandOpMode _opMode, DriveSubsystem _drive) {
        drive = _drive;
        opMode = _opMode;
        addRequirements(drive);
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){

        y = MyMath.applyDeadband(-Hw.gpDriver.getLeftY(), .05, 1.0);
        x = MyMath.applyDeadband(Hw.gpDriver.getLeftX(), .05, 1.0);
        z = MyMath.applyDeadband(-Hw.gpDriver.getRightX(), .05, 1.0);
        ang = drive.getRobotAngle();

        drive.driveCartesianIK(y,-x,z*.5,ang);
        opMode.telemetry.addData("x = ", -x);
        opMode.telemetry.addData("y = ", y);
        opMode.telemetry.addData("z = ", z);
    }

}
