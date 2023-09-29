package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Lib.Hw;

public class Claw {
    CRServo m_left;
    CRServo m_right;
    RevColorSensorV3 m_CSLeft;
    RevColorSensorV3 m_CSRight;

    CommandOpMode m_opMode;

    public Claw(CommandOpMode _opMode)
    {
        m_opMode = _opMode;
    }
    public void initHardware(){
        m_left = new CRServo(m_opMode.hardwareMap, Hw.s_claw_servoLeft);
        m_right = new CRServo(m_opMode.hardwareMap, Hw.s_claw_servoRight);
        m_CSLeft = m_opMode.hardwareMap.get(RevColorSensorV3.class, Hw.s_claw_CSLeft);
        m_CSRight = m_opMode.hardwareMap.get(RevColorSensorV3.class, Hw.s_claw_CSRight);

    }
    public void spinLeft(double _speed){
        m_left.set(_speed);
    }
    public void spinRight(double _speed){
        m_right.set(_speed);
    }
    public NormalizedRGBA getLeftColor(){
        return m_CSLeft.getNormalizedColors();
    }
    public NormalizedRGBA getRightColor(){
        return m_CSRight.getNormalizedColors();
    }
}
