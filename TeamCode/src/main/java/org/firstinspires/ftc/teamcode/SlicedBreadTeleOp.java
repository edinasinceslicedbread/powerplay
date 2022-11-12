package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SlicedBreadTeleOp extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static final boolean FIELD_CENTRIC = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "rightFront"),
                new Motor(hardwareMap, "leftRear"),
                new Motor(hardwareMap, "rightRear")
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        // create lift object
        LiftTool lift = new LiftTool();
        lift.init(hardwareMap);
        int liftTarget = 200;

        // create wrist object
        WristTool wrist = new WristTool();
        wrist.init(hardwareMap);

        // create intake object
        IntakeTool intake = new IntakeTool();
        intake.init(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {

            if(driverOp.getButton(GamepadKeys.Button.Y)) {
                liftTarget=3000;
            }

            if(driverOp.getButton(GamepadKeys.Button.X)) {
                liftTarget=2000;
            }

            if(driverOp.getButton(GamepadKeys.Button.B)) {
                liftTarget=1000;
            }

            if(driverOp.getButton(GamepadKeys.Button.A)) {
                liftTarget=200;
            }

            if(driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                wrist.moveAbsolute(0);
                telemetry.addLine("LEFT_BUMPER");
                telemetry.update();
            }

            if(driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                wrist.moveAbsolute(1);
                telemetry.addLine("RIGHT_BUMPER");
                telemetry.update();
            }

            if(driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)==1) {
                intake.moveAbsolute(0);
                telemetry.addLine("LEFT_TRIGGER");
                telemetry.update();
            }

            if(driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)==1) {
                intake.moveAbsolute(1);
                telemetry.addLine("RIGHT_TRIGGER");
                telemetry.update();
            }

            lift.moveAbsolute(liftTarget);

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
        }
    }
}
