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

@Autonomous(name="FirstAutoTest X", group="Mark")
public class SlicedBreadAuto extends LinearOpMode {

    AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();
    AutonomousOptions autonomousOptions = new AutonomousOptions();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    SampleMecanumDrive drive;
    Pose2d startPose;
    TrajectorySequence trajSeq;
    private float x,y,degrees=0;
    private int parkZone = 24; // Set this variable when we read the AprilTag
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

        if(autonomousOptions.getAllianceColor() == AutonomousOptions.AllianceColor.Blue) {
            if(autonomousOptions.getStartPosition() == AutonomousOptions.StartPosition.Right) {  // Blue Right
                x = -36;
                y = 64;
                degrees = -90;

                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(-36,12))
                        .turn(Math.toRadians(45))
                        .forward(6)
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
                        .back(6)
                        .turn(Math.toRadians(45))
                        .strafeLeft(parkZone)
                        .build();
            }
        } else {  // Red Alliance
            if(autonomousOptions.getStartPosition() == AutonomousOptions.StartPosition.Right) {  // Red Right
                x = 36;
                y = -64;
                degrees = 90;
                startPose = new Pose2d(x, y, Math.toRadians(degrees));
                drive.setPoseEstimate(startPose);
                trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(36,-12))
                        .turn(Math.toRadians(45))
                        .forward(6)
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
                        .back(6)
                        .turn(Math.toRadians(45))
                        .strafeLeft(parkZone)
                        .build();
            }
        }

        waitForStart();

        drive.followTrajectorySequence(trajSeq);
    }
}
