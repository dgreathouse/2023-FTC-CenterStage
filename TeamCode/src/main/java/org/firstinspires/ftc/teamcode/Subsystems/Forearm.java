package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;

public class Forearm {
    private CommandOpMode m_opMode;
    private MotorEx m_motor;

    public Forearm(CommandOpMode _opMode) {
        m_opMode = _opMode;
    }
    public void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.s_FA_Motor, Motor.GoBILDA.RPM_435);
        m_motor.setRunMode(Motor.RunMode.PositionControl);

    }
    public void setPosition(int _pos){
        m_motor.setTargetPosition(_pos);
    }
    public int getPosition(){
        return m_motor.getCurrentPosition();
    }


}
