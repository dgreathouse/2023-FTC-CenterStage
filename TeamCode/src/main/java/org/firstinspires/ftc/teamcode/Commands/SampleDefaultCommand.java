package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.SampleSubsystem;
// FIXME: Copy this class and SampleSubsystem to appropriate folders.
// FIXME: Rename by refactoring the file name to appropriate name
// FIXME: Refactor/Rename m_sample to appropriate name
public class SampleDefaultCommand extends CommandBase {
    SampleSubsystem m_sample;
    CommandOpMode m_opMode;
    public SampleDefaultCommand(CommandOpMode _opMode, SampleSubsystem _sample){
        m_opMode = _opMode;
        m_sample = _sample;

        addRequirements(m_sample);
    }
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        // Call methods in the subsystem to make the motors move
        m_sample.rotate(0);
    }
    @Override
    public void end(boolean _interrupted){
        // Set all motors to zero output state when this command ends
        m_sample.disableMotors();
    }
}
