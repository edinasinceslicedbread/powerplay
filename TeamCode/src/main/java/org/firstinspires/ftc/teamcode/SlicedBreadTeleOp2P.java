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
import com.arcrobotics.ftclib.util.MathUtils;
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

@TeleOp(name="SlicedBread TeleOp 2P", group="Iterative Opmode")
public class SlicedBreadTeleOp2P extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive = null;
    private RevIMU imu = null;
    private GamepadEx driverOp, toolOp = null;
    private LiftTool lift = null;
    private WristTool wrist = null;
    private IntakeTool intake = null;
    private int liftTarget,liftStartPos;
    private double intakeTarget,wristTarget;

    // drive constants
    double turbo = 0.6;
    private double speed_limit = 1.0;
    double LIMIT_RAMP = .75;

    // lift constants
    final int HIGH = 2900;
    final int MEDIUM = 2100;
    final int LOW = 1250;
    final int DRIVE = 0;
    final int MIN_WRIST = 350;
    final int LIFT_INCREMENT = 10;

    // intake constants
    final double CLOSED = .75;
    final double OPEN = 0;

    // wrist constants
    double wristEndTime;
    double WRIST_DELAY=750;

    final double FRONT = 0.03;
    final double BACK = 1.03;

    WristState wristState = WristState.NORMAL;

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
        if (toolOp.getButton(GamepadKeys.Button.Y)) {
            liftTarget = HIGH;
        }

        // Mid Height
        if (toolOp.getButton(GamepadKeys.Button.X)) {
            liftTarget = MEDIUM;
        }

        // Short Height
        if (toolOp.getButton(GamepadKeys.Button.B)) {
            liftTarget = LOW;
        }

        // Bottom
        if (toolOp.getButton(GamepadKeys.Button.A)) {
            liftTarget = DRIVE;
        }

        driverOp.readButtons();
        toolOp.readButtons();

        // Back and Front Toggle Wrist
        if (toolOp.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            wristTarget = (wristTarget == FRONT) ? BACK : FRONT;
            if (liftTarget > DRIVE + MIN_WRIST) {                   // Lift is in a safe position for wrist flip
                wristState = WristState.NORMAL;
            } else {    // Lift is unsafe, lift and flip and drop
                liftStartPos = liftTarget;
                wristState = WristState.LIFT;
            }
        }

        // Eject
        if (toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1) {
            intakeTarget = OPEN;
        }

        // Intake
        if (toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1) {
            intakeTarget = CLOSED;
        }

        // Manually adjusts lift
        liftTarget = liftTarget + (int)(toolOp.getLeftY()*LIFT_INCREMENT);
        liftTarget = MathUtils.clamp(liftTarget, DRIVE, HIGH);

        // move the tool parts
        lift.moveAbsolute(liftTarget);
        intake.moveAbsolute(intakeTarget);

        switch(wristState) {
            case NORMAL:
                wrist.moveAbsolute(wristTarget);
                break;
            case LIFT:
                liftTarget = MIN_WRIST;
                if (lift.getCurrentPosition()>=MIN_WRIST) {
                    wristState = WristState.FLIP;
                    wristEndTime = System.nanoTime() + 1E6 * WRIST_DELAY;
                }
                break;
            case FLIP:
                if (System.nanoTime() > wristEndTime) {
                    wristState = WristState.DROP;
                } else {
                    wrist.moveAbsolute(wristTarget);
                }
                break;
            case DROP:
                liftTarget = liftStartPos;
                if(lift.getCurrentPosition()==liftTarget) {
                    wristState = WristState.NORMAL;
                }
                break;
        }

        // calculate drive parameters
        turbo = 0.5 + (driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)/4) - (driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)/3);
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
        telemetry.addData("Speed Limit", "Speed Limit: " +speed_limit);
        telemetry.addData("Lift Target", "Lift Target: "+liftTarget);
        telemetry.addData("Lift Height", "Lift Height:"+lift.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        lift.moveAbsolute(0);
    }
}
