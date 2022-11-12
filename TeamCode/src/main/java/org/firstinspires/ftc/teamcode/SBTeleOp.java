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

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntakeTool;
import org.firstinspires.ftc.teamcode.LiftTool;
import org.firstinspires.ftc.teamcode.WristTool;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SB TeleOp", group="Iterative Opmode")
public class SBTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive = null;
    private RevIMU imu = null;
    private GamepadEx driverOp, toolOp = null;
    private LiftTool lift = null;
    private WristTool wrist = null;
    private IntakeTool intake = null;
    private int liftTarget,intakeTarget;

    // Change this to switch between FIELD_CENTRIC and Robot Centric
    static final boolean FIELD_CENTRIC = true;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        drive = new MecanumDrive(
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "rightFront"),
                new Motor(hardwareMap, "leftRear"),
                new Motor(hardwareMap, "rightRear")
        );

        imu = new RevIMU(hardwareMap);
        imu.init();

        // the extended gamepad object
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        // init Lift
        lift = new LiftTool();
        lift.init(hardwareMap);
        liftTarget = 300;
        intakeTarget = 0;

        // init Wrist
        wrist = new WristTool();
        wrist.init(hardwareMap);
        wrist.moveAbsolute(0);

        // init Intake
        intake = new IntakeTool();
        intake.init(hardwareMap);
        intake.moveAbsolute(intakeTarget);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Full Height
        if(driverOp.getButton(GamepadKeys.Button.Y)) {
            liftTarget=3000;
        }

        // Mid Height
        if(driverOp.getButton(GamepadKeys.Button.X)) {
            liftTarget=2000;
        }

        // Short Height
        if(driverOp.getButton(GamepadKeys.Button.B)) {
            liftTarget=1000;
        }

        // Bottom
        if(driverOp.getButton(GamepadKeys.Button.A)) {
            liftTarget=300;
        }

        // Front
        if(driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            wrist.moveAbsolute(0);
        }

        // Back
        if(driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            wrist.moveAbsolute(1);
        }

        // Intake
        if(driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)==1) {
            intakeTarget=0;
        }

        // Eject
        if(driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)==1) {
            liftTarget = 100;
            intakeTarget=1;
        }

        lift.moveAbsolute(liftTarget);
        intake.moveAbsolute(intakeTarget);

        if (!FIELD_CENTRIC) {
            drive.driveRobotCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX(),
                    false
            );
        } else {
            drive.driveFieldCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX(),
                    imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                    false
            );
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        lift.moveAbsolute(0);
    }

}
