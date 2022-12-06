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

@TeleOp(name="SlicedBread TeleOp TWO", group="Iterative Opmode")
public class SlicedBreadTeleOp1P extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive = null;
    private RevIMU imu = null;
    private GamepadEx driverOp = null;
    private LiftTool lift = null;
    private WristTool wrist = null;
    private IntakeTool intake = null;
    private int liftTarget;
    private double intakeTarget,wristTarget;

    // drive constants
    private double turbo = 0.5;
    private double speed_limit = 1.0;
    private final double TURBO_INCREMENT = 0.1;
    private final double LIMIT_RAMP= .75;

    // lift constants
    final int HIGH = 2900;
    final int MEDIUM = 2100;
    final int LOW = 1250;
    final int MIN_WRIST = 350;
    final int DRIVE = 0;
    final int LIFT_INCREMENT = 50;

    // intake constants
    final double CLOSED = .75;
    final double OPEN = 0;

    // wrist constants
    final double FRONT = 0;
    final double SIDE = 0.5;
    final double BACK = 1.0;

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
        intakeTarget = OPEN;

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
        if (driverOp.getButton(GamepadKeys.Button.Y)) {
            liftTarget = HIGH;
        }

        // Mid Height
        if (driverOp.getButton(GamepadKeys.Button.X)) {
            liftTarget = MEDIUM;
        }

        // Short Height
        if (driverOp.getButton(GamepadKeys.Button.B)) {
            liftTarget = LOW;
        }

        // Bottom
        if (driverOp.getButton(GamepadKeys.Button.A)) {
            liftTarget = DRIVE;
        }

        driverOp.readButtons();

        // Bump up
        if (driverOp.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            liftTarget = Math.min(HIGH, liftTarget + LIFT_INCREMENT);
        }

        // Bump down
        if (driverOp.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
            liftTarget = Math.max(0, liftTarget - LIFT_INCREMENT);
        }

        // Turbo up
        if (driverOp.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
            turbo = Math.min(1.0, turbo + TURBO_INCREMENT);
        }

        // Turbo down
        if (driverOp.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            turbo = Math.min(0, turbo - TURBO_INCREMENT);
        }

        // Front
        if (driverOp.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) && liftTarget >= MIN_WRIST) {
            wristTarget = BACK;
        }

        // Back
        if (driverOp.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER) && liftTarget >= MIN_WRIST) {
            wristTarget = FRONT;
        }

        // Middle
        if (driverOp.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) && driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER) && liftTarget >= MIN_WRIST) {
            wristTarget = SIDE;
        }

        // Eject
        if (driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1) {
            intakeTarget = OPEN;
        }

        // Intake
        if (driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1) {
            intakeTarget = CLOSED;
        }

        lift.moveAbsolute(liftTarget);
        intake.moveAbsolute(intakeTarget);
        wrist.moveAbsolute(wristTarget);

        speed_limit = 1-(((double)liftTarget/(double)HIGH) * LIMIT_RAMP);
        drive.setRange(-speed_limit, speed_limit);

        if (!FIELD_CENTRIC) {
            drive.driveRobotCentric(
                    driverOp.getLeftX() * turbo,
                    driverOp.getLeftY() * turbo,
                    driverOp.getRightX() * turbo,
                    false
            );
        } else {
            drive.driveFieldCentric(
                    driverOp.getLeftX() * turbo,
                    driverOp.getLeftY() * turbo,
                    driverOp.getRightX() * turbo,
                    imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                    true
            );
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Speed", "Turbo Factor: " + turbo);
        telemetry.addData("Speed Limit", "Speed Limit: " + speed_limit);
        telemetry.addData("Lift Height", "Lift Height: "+ liftTarget);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        lift.moveAbsolute(0);
    }
}
