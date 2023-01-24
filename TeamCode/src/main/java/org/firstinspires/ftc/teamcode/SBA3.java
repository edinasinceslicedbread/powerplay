package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="SBA3", group="Autonomous")
public class SBA3 extends LinearOpMode {

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

    // UNITS ARE METERS
    double tagsize = 0.0444;

    int numFramesWithoutDetection = 0;
    boolean tagFound = false;

    // april tag constants
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    // parking constants
    final double ZONE_ONE = 24;
    final double ZONE_TWO = 0;
    final double ZONE_THREE = -24;

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
        intake.moveAbsolute(AutonomousTrajectories.CLOSED);
        sleep(1000);

        // set start game lift position
        lift.setTargetPosition(AutonomousTrajectories.STACK);
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

        if(autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Right
                && autonomousConfiguration.getFirstDrop() == AutonomousOptions.FirstDrop.C2
                && autonomousConfiguration.getDropLocation() == AutonomousOptions.DropLocation.D3) {

                // C2 D3 Right
                telemetry.speak("C2 D3 Right Routine Begin");
                trajSeq = AutonomousTrajectories.trajectory_C2_D3_right(parkZone, drive, lift, wrist, intake);
        } else if(autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Left
                && autonomousConfiguration.getFirstDrop() == AutonomousOptions.FirstDrop.C2
                && autonomousConfiguration.getDropLocation() == AutonomousOptions.DropLocation.D3) {

                // C2 B3 Left CHANGE*****
                telemetry.speak("C2 B3 Left Routine Begin");
                trajSeq = AutonomousTrajectories.trajectory_C2_D3_left(parkZone, drive, lift, wrist, intake);
        } else if(autonomousConfiguration.getStartPosition() == AutonomousOptions.StartPosition.Right
                && autonomousConfiguration.getFirstDrop() == AutonomousOptions.FirstDrop.D2
                && autonomousConfiguration.getDropLocation() == AutonomousOptions.DropLocation.D3) {

                // D2 D3 Left
                telemetry.speak("Left Routine Begin");
                trajSeq = AutonomousTrajectories.trajectory_C2_D3_left(parkZone, drive, lift, wrist, intake);
        }
        drive.followTrajectorySequence(trajSeq);
    }
}
