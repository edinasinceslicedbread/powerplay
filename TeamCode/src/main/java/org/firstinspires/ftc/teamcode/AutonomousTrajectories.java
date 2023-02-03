package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public final class AutonomousTrajectories {

    // Starting points
    final static double START_LEFT_X = 41.25;
    final static double START_LEFT_Y = 64;
    final static double START_RIGHT_X = 32;
    final static double START_RIGHT_Y = 64;

    // coordinate autonomous constants
    final static double D3_X = 30.5;
    final static double D3_Y = 4.4;
    final static double B3_X = 32;
    final static double B3_Y = 5;
    final static double LEFT_D2_X = 15;
    final static double LEFT_D2_Y = 24; // check for accuracy
    final static double RIGHT_D2_X = 30.5;
    final static double RIGHT_D2_Y = 19.6; // check for accuracy
    final static double RIGHT_B2_X = 33.1;
    final static double RIGHT_B2_Y = 28.4; // check for accuracy
    final static double LEFT_B2_X = 33.1;
    final static double LEFT_B2_Y = 28.4; // check for accuracy

    final static double RIGHT_C2_X = 9.0;
    final static double RIGHT_C2_Y = 24.0;
    final static double LEFT_C2_X = 9.5;
    final static double LEFT_C2_Y = 24;

    // Cone stack coordinates
    final static double RIGHT_STACK_X = 62.5;
    final static double RIGHT_STACK_Y = 11;
    final static double LEFT_STACK_X = 63;
    final static double LEFT_STACK_Y = 11;

    // gripper constants
    final static double CLOSED = .6;
    final static double OPEN = 0;

    // arm height constants
    final static int HIGH = 2825;
    final static int MEDIUM = 2025;
    final static int LOW = 1175;
    final static int STACK = 400;
    final static int DRIVE = 0;

    final static int STACK_SAFE = 750;
    final static int CONE_HEIGHT = 75;

    // wrist constants
    final static double FRONT = 0.25;
    final static double BACK = 0.91;

    // wrist constants
    double wristEndTime;
    double WRIST_DELAY=750;

    // C2-D3 Right
    // Code Status: Yes
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_C2_D3(double parkZone, SampleMecanumDrive drive, DcMotor lift, WristTool wrist, IntakeTool intake) {

        // set start position
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;

        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to front
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around D1
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
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
                .splineToSplineHeading(new Pose2d(RIGHT_C2_X, -RIGHT_C2_Y, Math.toRadians(180)), Math.toRadians(90))
                // reposition wrist and drop
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                .setReversed(true)
                .setTangent(70)
                .splineToSplineHeading(new Pose2d(22, -12, Math.toRadians(180)), Math.toRadians(0))
                // lower lift to stack height
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+.75, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+1.5, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+2.25, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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

        return trajSeq;
    }

    // C2-B3 Left
    // Code Status: Yes
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_C2_B3(double parkZone, SampleMecanumDrive drive, DcMotor lift, WristTool wrist, IntakeTool intake) {
        // set start pose
        Pose2d startPose = new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to front
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around D1
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(90)), Math.toRadians(0))
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
                .splineToSplineHeading(new Pose2d(-LEFT_C2_X, -LEFT_C2_Y, Math.toRadians(0)), Math.toRadians(90))
                // reposition wrist and drop
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                //
                // CONE 1+1
                //
                // move to stack for new cone
                .setReversed(true)
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-18, -12, Math.toRadians(0)), Math.toRadians(180))
                // lower lift to stack height
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
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
                .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
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
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X-1, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
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
                .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
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
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X-2, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
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
                .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
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
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X-3, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
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

        return trajSeq;
    }

    // D2-D3 Right
    // Code Status: Yes
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_D2_D3(double parkZone, SampleMecanumDrive drive, DcMotor lift, WristTool wrist, IntakeTool intake) {

        // set start position
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;

        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around D1
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // raise lift to MID
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(MEDIUM);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                //
                // Cone 1
                //
                // drive to D2
                .splineToSplineHeading(new Pose2d(12, -30, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(LEFT_D2_X, -LEFT_D2_Y, Math.toRadians(180)), Math.toRadians(45))
                // reposition wrist and drop
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)
                // drive around D2
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(18, -10, Math.toRadians(180)), Math.toRadians(0))
                // lower lift to stack height
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })

                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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
                // reposition wrist to FRONT position
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
                // reposition wrist to FRONT position
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

                // lift to stack HIGH-3ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 3);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                //
                // Cone 1+4
                //
                // return to stack for cone 4
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
                // reposition wrist to FRONT position
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

        return trajSeq;
    }

    // B2-B3 Left
    // Code Status: Yes
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_B2_B3(double parkZone, SampleMecanumDrive drive, DcMotor lift, WristTool wrist, IntakeTool intake) {

        //To do: Change to Left routine
        //To do: Change comments to B2 B3
        //Current state - in right oriented routine - UNCHANGED thus far

        // set start position
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;

        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around D1
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(90)), Math.toRadians(0))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // raise lift to MEDIUM
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(MEDIUM);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                //
                // Cone 1
                //
                // drive to D2
                .splineToSplineHeading(new Pose2d(-12, -30, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-RIGHT_B2_X, -RIGHT_B2_Y, Math.toRadians(180)), Math.toRadians(135))
                // reposition wrist and drop
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)
                // drive around D2

                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(new Pose2d(-20, -12, Math.toRadians(180)), Math.toRadians(180))
                // lower lift to stack height
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })

                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(180)), Math.toRadians(180))

                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to D3
                .setReversed(true)
                .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-1ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 1);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+2
                //
                // return to stack for new cone
                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))

                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to B3
                .setReversed(true)
                .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-2ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 2);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))

                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to B3
                .setReversed(true)
                .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-3ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 3);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+4
                //
                // return to stack for cone 4
                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))

                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to B3
                .setReversed(true)
                .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)


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

        return trajSeq;
    }

    // D2-D2 Right
    // Code Status: No
    // Test Status: No
    // TODO: Implement Code
    // TODO: Test Code
    public static TrajectorySequence trajectory_D2_D2(double parkZone, SampleMecanumDrive drive, DcMotor lift, WristTool wrist, IntakeTool intake) {

        // set start position
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;

        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around D1
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // raise lift to MID
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(MEDIUM);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                //
                // Cone 1
                //
                // drive to D2
                .splineToSplineHeading(new Pose2d(12, -30, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(LEFT_D2_X, -LEFT_D2_Y, Math.toRadians(180)), Math.toRadians(45))
                // reposition wrist and drop
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)
                // drive around D2
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(18, -10, Math.toRadians(180)), Math.toRadians(0))
                // lower lift to stack height
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to D2
                .setReversed(true)
                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-1ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 1);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+2
                //
                // return to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+.75, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to D3
                .setReversed(true)
                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-2ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 2);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+1.5, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to D3
                .setReversed(true)
                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-2ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 3);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+3.5
                //
                // return to stack for cone 3
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+2.25, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

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

        return trajSeq;
    }

    // B2-B2 Left
    // Code Status: No
    // Test Status: No
    // TODO: Implement Code
    // TODO: Test Code
    public static TrajectorySequence trajectory_B2_B2(double parkZone, SampleMecanumDrive drive, DcMotor lift, WristTool wrist, IntakeTool intake) {

        // set start position
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;

        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around D1
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(90)), Math.toRadians(0))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // raise lift to MEDIUM
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(MEDIUM);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                //
                // Cone 1
                //
                // drive to D2
                .splineToSplineHeading(new Pose2d(-12, -30, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-RIGHT_B2_X, -RIGHT_B2_Y, Math.toRadians(180)), Math.toRadians(135))
                // reposition wrist and drop
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)
                // drive around D2

                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(new Pose2d(-20, -12, Math.toRadians(180)), Math.toRadians(180))
                // lower lift to stack height
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })

                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to D3
                .setReversed(true)
                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-1ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 1);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+2
                //
                // return to stack for new cone
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to D3
                .setReversed(true)
                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-2ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 2);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(HIGH);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drive to D3
                .setReversed(true)
                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // lift to stack HIGH-2ch
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK - CONE_HEIGHT * 3);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // flip wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))

                //
                // Cone 1+3.5
                //
                // return to stack for cone 3
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK_SAFE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                // lift to DRIVE
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(DRIVE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })

                // drive to middle of zone
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-36-parkZone,-12, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                .build();

        return trajSeq;
    }

    // C2-D2 Right
    // Code Status: Yes (:
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_C2_D2(double parkZone, SampleMecanumDrive drive, DcMotor lift, WristTool wrist, IntakeTool intake) {

        // set start position
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;

        // create trajectory sequence
        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to front
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around D1
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
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
                .splineToSplineHeading(new Pose2d(RIGHT_C2_X, -RIGHT_C2_Y, Math.toRadians(180)), Math.toRadians(90))
                // reposition wrist and drop
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                .setReversed(true)
                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(new Pose2d(18, -10, Math.toRadians(180)), Math.toRadians(0))
                // lower lift to stack height
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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
                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
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
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+.75, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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
                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
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
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+1.5, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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
                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
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
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+2.25, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
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

        return trajSeq;
    }

    // C2-B2 Left
    // Code Status: Yes (:
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_C2_B2(double parkZone, SampleMecanumDrive drive, DcMotor lift, WristTool wrist, IntakeTool intake) {

        // set start position
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;

        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to front
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around D1
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(90)), Math.toRadians(0))
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
                .splineToSplineHeading(new Pose2d(-LEFT_C2_X, -LEFT_C2_Y, Math.toRadians(0)), Math.toRadians(90))
                // reposition wrist and drop
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                .setReversed(true)
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-18, -12, Math.toRadians(0)), Math.toRadians(180))
                // lower lift to stack height
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(STACK);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))

                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
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
                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
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
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
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
                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
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
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
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
                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
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
                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
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

                // lift to DRIVE
                .addTemporalMarker(() -> {
                    lift.setTargetPosition(DRIVE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })

                // drive to middle of zone
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-36-parkZone,-12, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                .build();

        return trajSeq;
    }

}


