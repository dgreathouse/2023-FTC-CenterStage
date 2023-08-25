package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;

public class SampleSubsystem extends SubsystemBase {
    // Declare all local data for  the class, including motor/servos exclusively used by this class
    private CommandOpMode m_opMode;
    private MotorEx m_myMotor;
    // Define the class constructor which has the same name as the class with no return value
    public SampleSubsystem (CommandOpMode _opMode){
        m_opMode = _opMode;

        initHardware();
    }
    // Define methods that make the motor move
    public void rotate(double _speed){
        m_myMotor.set(_speed);
    }

    private void initHardware(){
        m_myMotor = new MotorEx(m_opMode.hardwareMap, Hw.s_m, Motor.GoBILDA.RPM_435);
        m_myMotor.setRunMode(Motor.RunMode.RawPower);
        m_myMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }
    @Override
    public void periodic(){
        m_opMode.telemetry.addData("S Vel", m_myMotor.getVelocity());
    }
    public void disableMotors(){
        m_myMotor.set(0);
    }
}

