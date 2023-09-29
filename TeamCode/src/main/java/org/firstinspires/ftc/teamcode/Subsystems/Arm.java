package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private CommandOpMode m_opMode;
    private Claw m_claw;
    private Shoulder m_shoulder;
    private Forearm m_forearm;

    public Arm(CommandOpMode _opMode){
        m_opMode = _opMode;
        initHardware();
    }
    private void initHardware(){
        m_claw = new Claw(m_opMode);
        m_shoulder = new Shoulder(m_opMode);
        m_forearm = new Forearm(m_opMode);
    }

    @Override
    public void periodic(){

    }
}
