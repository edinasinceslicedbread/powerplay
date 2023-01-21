package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="SBA2", group="Autonomous")
public class SBA2 extends LinearOpMode {

    // Menu initialization
    AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // Vision initialization
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;

    // autonomous cone stack coordinates
    final double RIGHT_STACK_X = 62.5;
    final double RIGHT_STACK_Y = 11;
    final double LEFT_STACK_X = 63;
    final double LEFT_STACK_Y = 11;

    // coordinate autonomous constants
    final double D3_X = 30.5;
    final double D3_Y = 4.4;
    final double B3_X = 32;
    final double B3_Y = 5;
    final double D2_X = 33.1;
    final double D2_Y = 28.4; // check for accuracy
    final double RIGHT_C2_X = 9.0;
    final double RIGHT_C2_Y = 24.0;
    final double LEFT_C2_X = 9.5;
    final double LEFT_C2_Y = 24;
    final double START_LEFT_X = 41.25;
    final double START_LEFT_Y = 64;
    final double START_RIGHT_X = 32;
    final double START_RIGHT_Y = 64;

    // UNITS ARE METERS
    double tagsize = 0.0444;

    int numFramesWithoutDetection = 0;
    boolean tagFound = false;

    // april tag constants
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    // gripper constants
    final double OPEN = 0;
    final double CLOSED = 1;

    // arm height constants
    final int HIGH = 2825;
    final int DRIVE = 0;
    final int STACK = 400;
    final int STACK_SAFE = 750;
    final int CONE_HEIGHT = 75;

    // wrist constants
    final double FRONT = 0.03;
    final double BACK = 1.03;

    // parking constants
    final double ZONE_ONE = 24;
    final double ZONE_TWO = 0;
    final double ZONE_THREE = -24;

    Pose2d startPose;
    TrajectorySequence trajSeq;

    @Override
    public void runOpMode() throws InterruptedException {

        // Game initialization
        double parkZone = ZONE_TWO; // Set this variable when we read the AprilTag
        boolean menuFlag = false;

        // init Intake
        IntakeTool intake = new IntakeTool();
        intake.init(hardwareMap);

        WristTool wrist = new WristTool();
        wrist.init(hardwareMap);

        // init lift
        DcMotor lift = hardwareMap.dcMotor.get("lift");

        // set up lift
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // init Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        // preload cone
        intake.moveAbsolute(CLOSED);
        sleep(1000);

        // set start game lift position
        lift.setTargetPosition(300);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        // init menu
        autonomousConfiguration.init(this.gamepad1, this.telemetry, hardwareMap.appContext);

        // start reading tags
        // run set up menu
        while (!menuFlag && !isStarted()) {
            menuFlag = autonomousConfiguration.init_loop();
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags! Yay!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    tagFound=true;

                    // loop through detected tags - there should be only one (:
                    for (AprilTagDetection detection : detections) {

                        //determine which zone to park in
                        if (detection.id==1) {
                            parkZone = ZONE_ONE;
                        } else if (detection.id==2) {
                            parkZone = ZONE_TWO;
                        } else if (detection.id==3) {
                            parkZone = ZONE_THREE;
                        }

                        //telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    }
                }

                telemetry.update();
            }
        }

        // check to make sure starting config is valid
        if (!autonomousConfiguration.getReadyToStart()) {
            telemetry.addData("Alert", "Not ready to start!");
            telemetry.speak("Not ready to start!");
            runtime.reset();
            while (runtime.seconds() < 2) {
            }
            requestOpModeStop();
        }
        runtime.reset();

        // wait for start button to be pressed
        waitForStart();

        // do a starting delay if requested
        double delay = runtime.seconds()+autonomousConfiguration.getDelayStartSeconds();
        while (runtime.seconds() < delay) {
        }

        if(autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Right) {  // Right
            telemetry.speak("Right Routine Begin");

            // set start position
            startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
            drive.setPoseEstimate(startPose);

            // create trajectory sequence
            trajSeq = drive.trajectorySequenceBuilder(startPose)
                    // reposition wrist to front
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    // drive around D1
                    .lineTo(new Vector2d(14, -60))
                    // raise lift to HIGH
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                     // drive to C2
                    .splineToLinearHeading(new Pose2d(RIGHT_C2_X, -RIGHT_C2_Y, Math.toRadians(180)), Math.toRadians(90))
                    // reposition wrist and drop
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(14, -12, Math.toRadians(180)))
                    // lower lift to stack height
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to BACK
                    .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                    // move to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                    .setReversed(false)
                    .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                    .waitSeconds(0.5)
                    // lift cone off of stack
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK_SAFE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to FRONT
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    // lift to HIGH
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // drive to D3
                    .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                    // open intake and back up
                    .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                    .waitSeconds(0.5)
                    .back(2)
                    // lift to stack HIGH-1ch
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK - CONE_HEIGHT * 1);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // flip wrist to BACK position
                    .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                    // return to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(RIGHT_STACK_X+1, -RIGHT_STACK_Y), Math.toRadians(0))
                    .setReversed(false)
                    .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                    .waitSeconds(0.5)
                    // lift cone off of stack
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK_SAFE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to FRONT position
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    // lift to HIGH
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // drive to D3
                    .splineTo(new Vector2d(D3_X+1, -D3_Y), Math.toRadians(135.00))
                    // open intake and back up
                    .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                    .waitSeconds(0.5)
                    .back(2) // actually backing up
                    // lift to stack HIGH-2ch
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK - CONE_HEIGHT * 2);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // flip wrist to BACK position
                    .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                    // return to stack for cone 3
                    .setReversed(true)
                    .splineTo(new Vector2d(RIGHT_STACK_X+1.5, -RIGHT_STACK_Y), Math.toRadians(0))
                    .setReversed(false)
                    .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                    .waitSeconds(0.5)
                    // lift cone off of stack
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK_SAFE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to FRONT position
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    // lift to HIGH
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // drive to D3
                    .splineTo(new Vector2d(D3_X+2, -D3_Y), Math.toRadians(135.00))
                    // open intake and back up
                    .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                    .waitSeconds(0.5)
                    .back(2)

                    // lift to DRIVE
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(DRIVE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    // drive to middle of zone
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(36-parkZone,-12, Math.toRadians(90)), Math.toRadians(0))
                    .setReversed(false)
                    .build();
        } else {  // Left
            // set start pose
            telemetry.speak("Left Routine Begin");
            startPose = new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90));
            drive.setPoseEstimate(startPose);

            trajSeq = drive.trajectorySequenceBuilder(startPose)
                    // reposition wrist to front
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    // drive around D1
                    .lineTo(new Vector2d(-14, -60))
                    // raise lift to HIGH
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    //
                    // Cone 1
                    //
                    // drive to C2
                    .splineToLinearHeading(new Pose2d(-LEFT_C2_X, -LEFT_C2_Y, Math.toRadians(0)), Math.toRadians(90))
                    // reposition wrist and drop
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(-14, -12, Math.toRadians(0)))
                    // lower lift to stack height
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to BACK
                    .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                    //
                    // CONE 1+1
                    //
                    // move to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                    .setReversed(false)
                    .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                    .waitSeconds(0.5)
                    // lift cone off of stack
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK_SAFE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to FRONT
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    // lift to HIGH
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // drive to B3
                    .splineToLinearHeading(new Pose2d(-B3_X, -B3_Y, Math.toRadians(45)), Math.toRadians(45))
                    // open intake and back up
                    .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                    .waitSeconds(0.5)

                    // lift to stack HIGH-1ch
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK - CONE_HEIGHT * 1);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // flip wrist to BACK position
                    .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                    //
                    // Cone 1+2
                    //
                    // return to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-LEFT_STACK_X-1, -LEFT_STACK_Y), Math.toRadians(180))
                    .setReversed(false)
                    .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                    .waitSeconds(0.5)
                    // lift cone off of stack
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK_SAFE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to FRONT position
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    // lift to HIGH
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // drive to B3
                    .splineToLinearHeading(new Pose2d(-B3_X, -B3_Y, Math.toRadians(45)), Math.toRadians(45))
                    // open intake and back up
                    .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                    .waitSeconds(0.5)

                    // lift to stack HIGH-2ch
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK - CONE_HEIGHT * 2);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // flip wrist to BACK position
                    .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                    //
                    // Cone 1+3
                    //
                    // return to stack for cone 3
                    .setReversed(true)
                    .splineTo(new Vector2d(-LEFT_STACK_X-2, -LEFT_STACK_Y), Math.toRadians(180))
                    .setReversed(false)
                    .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                    .waitSeconds(0.5)
                    // lift cone off of stack
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK_SAFE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to FRONT position
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                    // lift to HIGH
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // drive to D3
                    .splineToLinearHeading(new Pose2d(-B3_X, -B3_Y, Math.toRadians(45)), Math.toRadians(45))
                    // open intake and back up
                    .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                    .waitSeconds(0.5)

                    // lift to stack HIGH-2ch
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK - CONE_HEIGHT * 3);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // flip wrist to BACK position
                    .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                    //
                    // Cone 1+3.5
                    //
                    // return to stack for cone 3
                    .setReversed(true)
                    .splineTo(new Vector2d(-LEFT_STACK_X-3, -LEFT_STACK_Y), Math.toRadians(180))
                    .setReversed(false)
                    .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                    .waitSeconds(0.5)
                    // lift cone off of stack
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(STACK_SAFE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })
                    // reposition wrist to FRONT position
                    .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                    // lift to stack DRIVE
                    .addTemporalMarker(() -> {
                        lift.setTargetPosition(DRIVE);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    // drive to middle of zone 2
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-36 - parkZone,-12, Math.toRadians(90)), Math.toRadians(0))
                    .setReversed(false)
                    .build();
        }

        drive.followTrajectorySequence(trajSeq);
    }
}
