package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name="SlicedBreadAuto", group="Autonomous")
public class SlicedBreadAuto extends LinearOpMode {

    // Menu initialization
    AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();
    //AutonomousOptions autonomousOptions = new AutonomousOptions();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

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

    // UNITS ARE METERS
    double tagsize = 0.0444;

    int numFramesWithoutDetection = 0;
    boolean tagFound = false;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    // Roadrunner Initialization
    SampleMecanumDrive drive;
    private LiftTool lift = null;
    private IntakeTool intake = null;

    Pose2d startPose;
    TrajectorySequence trajSeq;

    // Game initialization
    private float x,y,degrees=0;
    private double parkZone = 24; // Set this variable when we read the AprilTag
    boolean menuFlag = false;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        autonomousConfiguration.init(this.gamepad1, this.telemetry, hardwareMap.appContext);

        while (!menuFlag && !isStarted()) {
            menuFlag = autonomousConfiguration.init_loop();
            telemetry.update();
        }

        if (!autonomousConfiguration.getReadyToStart()) {
            telemetry.addData("Alert", "Not ready to start!");
            telemetry.speak("Not ready to start!");
            runtime.reset();
            while (runtime.seconds() < 2) {
            }
            requestOpModeStop();
        }
        runtime.reset();

        // init Lift
        lift = new LiftTool();
        lift.init(hardwareMap);

        // init Intake
        intake = new IntakeTool();
        intake.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();

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
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    tagFound=true;

                    for (AprilTagDetection detection : detections) {

                        if (detection.id==1) {
                            parkZone = 24;
                        } else if (detection.id==2) {
                            parkZone = 0.1;
                        } else if (detection.id==3) {
                            parkZone = -24;
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

        if(autonomousConfiguration.getAlliance() == AutonomousOptions.AllianceColor.Blue) {
            if(autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Right) {  // Blue Right
                x = -36;
                y = 64;
                degrees = -90;

                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(-36,12))
                        .turn(Math.toRadians(45))
                        .forward(6)
                        .addTemporalMarker(() -> lift.setTargetPosition(2950))
                        .forward(2)
                        .addTemporalMarker(() -> intake.moveAbsolute(0))
                        .back(2)
                        .addTemporalMarker(() -> lift.setTargetPosition(150))
                        .back(6)
                        .turn(Math.toRadians(-45))
                        .strafeLeft(parkZone)
                        .build();
            } else {  // Blue Left
                x = 36;
                y = 64;
                degrees = -90;

                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(36,12))
                        .turn(Math.toRadians(-45))
                        .forward(6)
                        .addTemporalMarker(() -> lift.setTargetPosition(2950))
                        .forward(2)
                        .addTemporalMarker(() -> intake.moveAbsolute(0))
                        .back(2)
                        .addTemporalMarker(() -> lift.setTargetPosition(150))
                        .back(6)
                        .turn(Math.toRadians(45))
                        .strafeLeft(parkZone)
                        .build();
            }
        } else {  // Red Alliance
            if(autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Right) {  // Red Right
                x = 36;
                y = -64;
                degrees = 90;
                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(36,-12))
                        .turn(Math.toRadians(45))
                        .forward(6)
                        .addTemporalMarker(() -> lift.setTargetPosition(2950))
                        .forward(2)
                        .addTemporalMarker(() -> intake.moveAbsolute(0))
                        .back(2)
                        .addTemporalMarker(() -> lift.setTargetPosition(150))
                        .back(6)
                        .turn(Math.toRadians(-45))
                        .strafeLeft(parkZone)
                        .build();
            } else {  // Red Left
                x = -36;
                y = -64;
                degrees = 90;
                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(-36,-12))
                        .turn(Math.toRadians(-45))
                        .forward(6)
                        .addTemporalMarker(() -> lift.setTargetPosition(2950))
                        .forward(2)
                        .addTemporalMarker(() -> intake.moveAbsolute(0))
                        .back(2)
                        .addTemporalMarker(() -> lift.setTargetPosition(150))
                        .back(6)
                        .turn(Math.toRadians(45))
                        .strafeLeft(parkZone)
                        .build();
            }
        }

        drive.followTrajectorySequence(trajSeq);
    }
}
