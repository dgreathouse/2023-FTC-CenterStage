package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class Hw {
    public static MotorEx flDrive;
    public static MotorEx frDrive;
    public static MotorEx bDrive;
    public static MotorEx brDrive;

    public static GamepadEx gpOperator, gpDriver;
    public static IMU imu;
    CommandOpMode opMode;
    public Hw(CommandOpMode _opMode) {
        opMode = _opMode;
    }
    public void init(){
        flDrive = new MotorEx(opMode.hardwareMap, "l", Motor.GoBILDA.RPM_435);
        flDrive.setInverted(false);
        flDrive.setRunMode(Motor.RunMode.RawPower);
        flDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        flDrive.encoder.setDirection(Motor.Direction.REVERSE);

        frDrive = new MotorEx(opMode.hardwareMap, "r", Motor.GoBILDA.RPM_435);
        frDrive.setInverted(false);
        frDrive.setRunMode(Motor.RunMode.RawPower);
        frDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        frDrive.encoder.setDirection(Motor.Direction.REVERSE);

        bDrive = new MotorEx(opMode.hardwareMap, "b", Motor.GoBILDA.RPM_435);
        bDrive.setInverted(false);
        bDrive.setRunMode(Motor.RunMode.RawPower);
        bDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bDrive.setDistancePerPulse(k.DRIVE.InchPerCount);
        bDrive.encoder.setDirection(Motor.Direction.REVERSE);


        gpDriver = new GamepadEx(opMode.gamepad1);
        gpOperator = new GamepadEx(opMode.gamepad2);
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu = opMode.hardwareMap.get(BHI260IMU.class,"imu");

      //  imu.init();


        opMode.telemetry.addData(">", "Hardware Initialized");
    }
}
