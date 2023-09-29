package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;

public class Shoulder {

    private  MotorEx m_motor;
    private CommandOpMode m_opMode;

    public Shoulder(CommandOpMode _opMode) {
        m_opMode = _opMode;
    }

    public void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.s_SH_m, MotorEx.GoBILDA.RPM_435);
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setInverted(false);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.setPositionCoefficient(1.0);
    }
    public void setoPosition(int _pos){
        m_motor.setTargetPosition(_pos);
    }
    public int getoPosition(){
       return  m_motor.getCurrentPosition();
    }
}
