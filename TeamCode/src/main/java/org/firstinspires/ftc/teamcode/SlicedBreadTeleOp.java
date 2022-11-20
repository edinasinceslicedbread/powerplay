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

@TeleOp(name="SlicedBread TeleOp", group="Iterative Opmode")
public class SlicedBreadTeleOp extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive = null;
    private RevIMU imu = null;
    private GamepadEx driverOp, toolOp = null;
    private LiftTool lift = null;
    private WristTool wrist = null;
    private IntakeTool intake = null;
    private int liftTarget;
    private double intakeTarget,wristTarget;

    private final double turbo = 0.50;
    private int intakeState = 0;

    final int HIGH = 2900;
    final int MEDIUM = 2100;
    final int LOW = 1250;
    final int DRIVE = 300;
    final int PICKUP = 125;

    final double CLOSED = 1.0;
    final double OPEN = 0;
    final double FRONT = 1.0;
    final double BACK = 0;

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
        liftTarget = DRIVE;

        // init Wrist
        wrist = new WristTool();
        wrist.init(hardwareMap);
        wristTarget=FRONT;

        // init Intake
        intake = new IntakeTool();
        intake.init(hardwareMap);
        intake.moveAbsolute(OPEN);
        intakeTarget = 0;

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
            liftTarget=HIGH;
            intakeState=0;
        }

        // Mid Height
        if(driverOp.getButton(GamepadKeys.Button.X)) {
            liftTarget=MEDIUM;
            intakeState=0;
        }

        // Short Height
        if(driverOp.getButton(GamepadKeys.Button.B)) {
            liftTarget=LOW;
            intakeState=0;
        }

        // Bottom
        if(driverOp.getButton(GamepadKeys.Button.A)) {
            liftTarget=DRIVE;
            intakeState=0;
        }

        // Front
        if(driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER) && liftTarget >= DRIVE) {
            wristTarget=BACK;
        }

        // Back
        if(driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER) && liftTarget >= DRIVE) {
            wristTarget=FRONT;
        }

        // Middle
        if(driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER) && driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER) && liftTarget >= DRIVE) {
            wristTarget=0.5;
        }

        // Eject
        if(driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)==1) {
            intakeState=-1;
        }

        // Intake
        if(driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)==1 && liftTarget <= LOW) {
            intakeState=1;
        }

        if (intakeState==0) {           // normal operation
            lift.moveAbsolute(liftTarget);
            intake.moveAbsolute(intakeTarget);
        } else if (intakeState==1) {    // open the grip
            intake.moveAbsolute(OPEN);

            intakeTarget = OPEN;
            intakeState = 2;
        } else if (intakeState==2) {    // lower to position
            lift.moveAbsolute(70);
            liftTarget=PICKUP;
            if (lift.getCurrentPosition() < PICKUP) {
                intakeState = 3;
            }
        } else if (intakeState==3) {    // close grip
            intake.moveAbsolute(CLOSED);
            intakeTarget = CLOSED;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                intakeState=0;
            }
            intakeState=4;
        } else if (intakeState==4) {        // safe drive height
            liftTarget = DRIVE;
            intakeState = 0;
        } else if (intakeState==-1) {       // Ejecting
            // check if we are in low position
            // if yes, move down before ejecting
            // if no, eject
            if (liftTarget==DRIVE) {
                liftTarget = PICKUP;               // go to bottom
                intakeState = -2;
            } else {
                intake.moveAbsolute(OPEN);
                intakeState = 0;
                intakeTarget = OPEN;
            }
        } else if (intakeState==-2) {        // 2nd stage of low eject
            intake.moveAbsolute(OPEN);     // open intake
            intakeTarget=OPEN;
            intakeState = -3;
        } else if (intakeState==-3) {       // Last step - go back to drive height
            liftTarget=DRIVE;
            intakeState = 0;
        }

        wrist.moveAbsolute(wristTarget);

        if (!FIELD_CENTRIC) {
            drive.driveRobotCentric(
                    driverOp.getLeftX()*turbo,
                    driverOp.getLeftY()*turbo,
                    driverOp.getRightX()*turbo,
                    false
            );
        } else {
            drive.driveFieldCentric(
                    driverOp.getLeftX()*turbo,
                    driverOp.getLeftY()*turbo,
                    driverOp.getRightX()*turbo,
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
        lift.moveAbsolute(20);
    }
}
