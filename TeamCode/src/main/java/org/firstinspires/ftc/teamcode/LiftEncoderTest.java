package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// hi dad! (:  - E
@Disabled
@Autonomous(name="Lift Encoder Test", group="Mark")
public class LiftEncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor liftFront = null;
    private DcMotor liftRear = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        liftFront  = hardwareMap.get(DcMotor.class, "lift");
        liftRear = hardwareMap.get(DcMotor.class, "liftRear");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        liftFront.setDirection(DcMotor.Direction.FORWARD);
        liftRear.setDirection(DcMotor.Direction.FORWARD);

        liftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

            while (opModeIsActive() &&
                    (runtime.seconds() < 30)) {

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Lift Front", "%7d", liftFront.getCurrentPosition());
                telemetry.addData("Lift Rear", "%7d", liftRear.getCurrentPosition());
                telemetry.update();
            }

            // Send calculated power to wheels
            liftFront.setPower(0);
            liftRear.setPower(0);

            // Turn off RUN_TO_POSITION
            liftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
}

