/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.SampleDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SampleSubsystem;


/**

 */

@TeleOp(name="TeleOp 1", group="Linear Opmode")

public class TeleOpMode_Linear extends CommandOpMode {
    ElapsedTime m_elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double m_timerAvg = 0.0;
    double m_timerCnt = 0;
    private Hw hw;
    private DriveSubsystem m_drive;
    private SampleSubsystem m_sample;  // FIXME: create the correct robot subsystems

    // Declare OpMode members.

    @Override
    public void initialize() {
        hw = new Hw(this);
        hw.init();

        // Create subsystems
        m_drive = new DriveSubsystem(this);
        m_sample = new SampleSubsystem(this); // FIXME: create the correct robot subsystems

        // Create Commands
        DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(this, m_drive);
        SampleDefaultCommand sampleDefaultCommand = new SampleDefaultCommand(this, m_sample);// FIXME: create the correct robot Commands

        // Set Default Commands
        m_drive.setDefaultCommand(driveDefaultCommand);
        m_sample.setDefaultCommand(sampleDefaultCommand);// FIXME: create the correct robot Commands

        // Set up buttons
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> m_drive.resetGyro(), m_drive));
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> m_drive.toggleIsFieldOriented(), m_drive));

    }

    @Override
    public void runOpMode() throws InterruptedException{
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            // Calculate the run rate of this loop
            m_timerAvg += m_elapsedTimer.seconds();
            if(m_timerCnt++ >= 10){
                telemetry.addData("LoopRate = ", m_timerAvg/10);
                m_timerAvg = 0.0; m_timerCnt = 0;
            }
            telemetry.update();
            // wait till timer is > 50ms to try an create a stable run rate
            while(m_elapsedTimer.seconds() < 0.05){} m_elapsedTimer.reset();
        }
        reset();
    }
}
