package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

public class AutoStopOpModeCommand extends CommandBase {
    CommandOpMode m_opMode;

    public AutoStopOpModeCommand(CommandOpMode _opMode){
        m_opMode = _opMode;
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){

    }
    @Override
    public boolean isFinished(){
        m_opMode.requestOpModeStop();
        return true;
    }
}
