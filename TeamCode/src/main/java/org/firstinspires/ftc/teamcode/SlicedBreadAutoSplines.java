package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.AutonomousOptions;

import java.util.ArrayList;

@Autonomous(name="SlicedBreadAutoSplines", group="Autonomous")
public class SlicedBreadAutoSplines extends LinearOpMode {

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

    final double RIGHT_DROPX = 8.0;
    final double RIGHT_DROPY = 30.0;
    final double LEFT_DROPX = 6.25;
    final double LEFT_DROPY = 31.75;

    // UNITS ARE METERS
    double tagsize = 0.0444;

    int numFramesWithoutDetection = 0;
    boolean tagFound = false;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    final double OPEN = 0;
    final double CLOSED = 1;

    final int HIGH = 2825;
    final int DRIVE = 0;

    final double FRONT = 0.03;
    final double SIDE = 0.53;
    final double BACK = 1.03;

    final double ZONE_ONE = 49;
    final double ZONE_TWO = 25;
    final double ZONE_THREE = 0.01;

    Pose2d startPose, startPose0;
    TrajectorySequence trajSeq,trajSeq0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Game initialization
        double x,y,degrees;
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

        // start reading tags
        while (opModeIsActive() && !tagFound) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

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

                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                    }
                }

                telemetry.update();
            }
        }

        // init Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        autonomousConfiguration.init(this.gamepad1, this.telemetry, hardwareMap.appContext);

        // run set up menu
        telemetry.speak("Hello, My Name is Bread!");
        while (!menuFlag && !isStarted()) {
            menuFlag = autonomousConfiguration.init_loop();
            telemetry.update();
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

        if(autonomousConfiguration.getAlliance() == AutonomousOptions.AllianceColor.Blue) {
            if(autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Right) {  // Blue Right
                x = -32;
                y = 64;
                degrees = -90;
                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                        .waitSeconds(1)
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(300);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .lineTo(new Vector2d(-16, 60))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(HIGH);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .splineToLinearHeading(new Pose2d(-RIGHT_DROPX, RIGHT_DROPY, Math.toRadians(-45)), Math.toRadians(-90))
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> intake.moveAbsolute(OPEN))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(-12,36, Math.toRadians(-90)))
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(DRIVE);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .waitSeconds(1)
                        .strafeLeft(parkZone-48.99) // flip it!
                        .build();

            } else {  // Blue Left
                x = 41.25;
                y = 64;
                degrees = -90;
                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                        .waitSeconds(1)
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(300);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .lineTo(new Vector2d(16, 60))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(HIGH);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .splineToLinearHeading(new Pose2d(LEFT_DROPX, LEFT_DROPY, Math.toRadians(-135)), Math.toRadians(-135))
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> intake.moveAbsolute(OPEN))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(12,36, Math.toRadians(-90)))
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(DRIVE);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .waitSeconds(1)
                        .strafeLeft(parkZone) // flip it!
                        .build();
            }
        } else {  // Red Alliance
            if(autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Right) {  // Red Right
                x = 32;
                y = -64;
                degrees = 90;
                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                        .waitSeconds(1)
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(300);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .lineTo(new Vector2d(16, -60))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(HIGH);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .splineToLinearHeading(new Pose2d(RIGHT_DROPX, -RIGHT_DROPY, Math.toRadians(135)), Math.toRadians(90))
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .waitSeconds(1)
                        .addTemporalMarker(() -> intake.moveAbsolute(OPEN))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(12,-36, Math.toRadians(90)))
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(DRIVE);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .waitSeconds(1)
                        .strafeLeft(parkZone-48.99) // flip it!
                        .build();
            } else {  // Red Left
                x = -41.25;
                y = -64;
                degrees = 90;
                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                        .waitSeconds(1)
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(300);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .lineTo(new Vector2d(-16, -60))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(HIGH);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .splineToLinearHeading(new Pose2d(-LEFT_DROPX, -LEFT_DROPY, Math.toRadians(45)), Math.toRadians(45))
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> intake.moveAbsolute(OPEN))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(-12,-36, Math.toRadians(90)))
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .addTemporalMarker(() -> {
                            lift.setTargetPosition(DRIVE);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                        })
                        .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                        .waitSeconds(1)
                        .strafeLeft(parkZone) // flip it!
                        .build();

            }
        }

        drive.followTrajectorySequence(trajSeq);
    }
}
