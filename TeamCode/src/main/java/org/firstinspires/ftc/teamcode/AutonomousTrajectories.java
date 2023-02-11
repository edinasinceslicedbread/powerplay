// elizabeth is da best
// thats it
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public final class AutonomousTrajectories {

    // Starting points
    final static double START_LEFT_X = 41.75;
    final static double START_LEFT_Y = 64;
    final static double START_RIGHT_X = 32;
    final static double START_RIGHT_Y = 64;

    // coordinate autonomous constants
    final static double B3_X = 35.5;
    final static double B3_Y = 7.5;

    final static double D3_X = 32;
    final static double D3_Y = 5.5;

    final static double LEFT_D2_X = 18;
    final static double LEFT_D2_Y = 24; // check for accuracy

    final static double RIGHT_D2_X = 32;
    final static double RIGHT_D2_Y = 18.5; // check for accuracy

    final static double RIGHT_B2_X = 18.5;
    final static double RIGHT_B2_Y = 26.5; // check for accuracy
    final static double LEFT_B2_X = 33.5;
    final static double LEFT_B2_Y = 18; // check for accuracy

    final static double RIGHT_C2_X = 8.5;
    final static double RIGHT_C2_Y = 22.5;
    final static double LEFT_C2_X = 9.5;
    final static double LEFT_C2_Y = 25;

    // Cone stack coordinates
    final static double RIGHT_STACK_X = 63.5;
    final static double RIGHT_STACK_Y = 13;
    final static double LEFT_STACK_X = 66;
    final static double LEFT_STACK_Y = 14.5;

    // gripper constants
    final static double CLOSED = .7;
    final static double OPEN = .1;

    // arm height constants
    final static int HIGH = 2775;
    final static int MEDIUM = 1950;
    final static int LOW = 1175;
    final static int STACK = 400;
    final static int DRIVE = 0;
    final static int INIT = 50;

    final static int STACK_SAFE = 750;
    final static int CONE_HEIGHT = 90;

    // wrist constants
    final static double FRONT = 0.25;
    final static double BACK = 0.91;

    static Pose2d finalPose;

    // C2-D3 Right
    // Code Status: Yes
    // Comment Status: Yes
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_C2_D3(SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {

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
                .splineToSplineHeading(new Pose2d(16, -58, Math.toRadians(90)), Math.toRadians(135))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                //
                // Cone 1
                //
                // drive to C2
                .splineToSplineHeading(new Pose2d(RIGHT_C2_X+4, -(RIGHT_C2_Y+12), Math.toRadians(180)), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(RIGHT_C2_X, -RIGHT_C2_Y, Math.toRadians(180)), Math.toRadians(180))

                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.1)
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> run_Lift(STACK, lift, liftRear))

                .setReversed(true)
                //
                // CONE 1+1
                //
                // move to stack for new cone
                .setTangent(70)
                .splineToSplineHeading(new Pose2d(24, -12, Math.toRadians(180)), Math.toRadians(0))
                // lower lift to stack height
                // reposition wrist to BACK

                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X + 0.5, -(RIGHT_STACK_Y), Math.toRadians(180)), Math.toRadians(0))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to D3
                .splineTo(new Vector2d(D3_X - 1.0, -(D3_Y - 1.0)), Math.toRadians(135.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.2)

                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.2)

                // lift to STACK-1ch
                .addTemporalMarker(() -> run_Lift(STACK - CONE_HEIGHT, lift, liftRear))

                //
                // Cone 1+2
                //
                // return to stack for new cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+1.5, -(RIGHT_STACK_Y + 1), Math.toRadians(180)), Math.toRadians(0))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to D3
                .splineTo(new Vector2d(D3_X, -(D3_Y)), Math.toRadians(135.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.2)

                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.2)

                // lift to STACK-2ch
                .addTemporalMarker(() -> run_Lift(STACK - (CONE_HEIGHT * 2), lift, liftRear))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X + 2.5, -(RIGHT_STACK_Y + 1), Math.toRadians(180)), Math.toRadians(0))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to D3
                .splineTo(new Vector2d(D3_X, -(D3_Y)), Math.toRadians(135.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.2)

                .build();

        finalPose = new Pose2d((D3_X), -(D3_Y), Math.toRadians(135.00));

        return trajSeq;
    }

    // C2-B3 Left
    // Code Status: Yes
    // Comment Status: Yes
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_C2_B3(SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {
        // set start pose
        Pose2d startPose = new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to front
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around B1
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(90)), Math.toRadians(0))
                // raise lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                //
                // Cone 1
                //
                // drive to C2
                .splineToSplineHeading(new Pose2d(-(LEFT_C2_X+4), -(LEFT_C2_Y+12), Math.toRadians(0)), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(-LEFT_C2_X, -LEFT_C2_Y, Math.toRadians(0)), Math.toRadians(0))

                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.1)
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> run_Lift(STACK, lift, liftRear))

                .setReversed(true)
                //
                // CONE 1+1
                //
                // move to stack for new cone
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(0)), Math.toRadians(180))

                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -(LEFT_STACK_Y), Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B3
                .splineTo(new Vector2d(-(B3_X), -(B3_Y)), Math.toRadians(45.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.1)

                // lift to STACK-1ch
                .addTemporalMarker(() -> run_Lift(STACK - CONE_HEIGHT, lift, liftRear))

                //
                // Cone 1+2
                //
                // return to stack for new cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X + 0.5), -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B3
                .splineTo(new Vector2d(-(B3_X), -B3_Y), Math.toRadians(45.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.5)

                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.1)

                // lift to STACK-2ch
                .addTemporalMarker(() -> run_Lift(STACK - (CONE_HEIGHT * 2), lift, liftRear))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X + 1), -(LEFT_STACK_Y), Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B3
                .splineTo(new Vector2d(-(B3_X), -(B3_Y)), Math.toRadians(45.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                .build();

                finalPose = new Pose2d(-(B3_X), -(B3_Y), Math.toRadians(45.00));

        return trajSeq;
    }

    // D2-D3 Right
    // Code Status: Yes
    // Comment Status: No
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_D2_D3(SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {
        // set start position
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;
        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around B1
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(15, -57, Math.toRadians(90)), Math.toRadians(135))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                //
                // Cone 1
                //
                // drive to D2
                .splineToSplineHeading(new Pose2d((LEFT_D2_X-4), -(LEFT_D2_Y+10), Math.toRadians(0)), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(LEFT_D2_X, -LEFT_D2_Y, Math.toRadians(0)), Math.toRadians(45))
                // reposition wrist and drop
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)
                // drive around D2

                .setTangent(Math.toRadians(110))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(12, -10, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(false)
                // lower lift to stack height
                .addTemporalMarker(() -> run_Lift(STACK, lift, liftRear))
                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -(RIGHT_STACK_Y-3), Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(D3_X - 1, -(D3_Y)), Math.toRadians(135.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                // lift to STACK-1ch
                .addTemporalMarker(() -> run_Lift(STACK - CONE_HEIGHT, lift, liftRear))
                //
                // Cone 1+2
                //
                // return to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+1, -(RIGHT_STACK_Y-3), Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(D3_X, -(D3_Y)), Math.toRadians(135.00))
                .setReversed(false)
                // open intake
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                // lift to STACK-2ch
                .addTemporalMarker(() -> run_Lift(STACK - (CONE_HEIGHT * 2), lift, liftRear))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+2, -(RIGHT_STACK_Y-3), Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(D3_X + 1.5, -(D3_Y + 1.5)), Math.toRadians(135.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                .build();

        finalPose = new Pose2d(D3_X, -(D3_Y), Math.toRadians(-45.00));

        return trajSeq;
    }

    // B2-B3 Left
    // Code Status: Yes
    // Comment Status: No
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_B2_B3(SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {
        // set start position
        Pose2d startPose = new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;
        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around B1
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(90)), Math.toRadians(0))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                //
                // Cone 1
                //
                // drive to D2
                .splineToSplineHeading(new Pose2d(-(RIGHT_B2_X-4), -(RIGHT_B2_Y+10), Math.toRadians(180)), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(-RIGHT_B2_X, -RIGHT_B2_Y, Math.toRadians(180)), Math.toRadians(135))
                // reposition wrist and drop
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)
                // drive around D2

                .setTangent(Math.toRadians(50))
                .splineToSplineHeading(new Pose2d(-16, -14, Math.toRadians(180)), Math.toRadians(180))
                // lower lift to stack height
                .addTemporalMarker(() -> run_Lift(STACK, lift, liftRear))
                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X - 1), -(LEFT_STACK_Y+1), Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(-(B3_X - 2), -(B3_Y+1)), Math.toRadians(45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                // lift to STACK-1ch
                .addTemporalMarker(() -> run_Lift(STACK - CONE_HEIGHT, lift, liftRear))
                //
                // Cone 1+2
                //
                // return to stack for new cone
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X - 1), -(LEFT_STACK_Y+1), Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(-(B3_X - 2), -(B3_Y + 1)), Math.toRadians(45.00))

                .setReversed(false)
                // open intake
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                // lift to STACK-2ch
                .addTemporalMarker(() -> run_Lift(STACK - (CONE_HEIGHT * 2), lift, liftRear))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X - 0.5), -(LEFT_STACK_Y+0.5), Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(-(B3_X - 2.5), -(B3_Y+1)), Math.toRadians(45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                .build();

        finalPose = new Pose2d(-(B3_X), -(B3_Y+1), Math.toRadians(225.00));

        return trajSeq;
    }

    // D2-D2 Right
    // Code Status: No
    // Comment Status: Yes
    // Test Status: No
    // TODO: Implement Code
    // TODO: Test Code
    public static TrajectorySequence trajectory_D2_D2(SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {
        Pose2d startPose = new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;
        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around B1
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(15, -57, Math.toRadians(90)), Math.toRadians(135))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                //
                // Cone 1
                //
                // drive to D2
                .splineToSplineHeading(new Pose2d((LEFT_D2_X-4), -(LEFT_D2_Y+10), Math.toRadians(0)), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(LEFT_D2_X, -LEFT_D2_Y, Math.toRadians(0)), Math.toRadians(45))
                // reposition wrist and drop
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)
                // drive around D2

                .setTangent(Math.toRadians(110))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(12, -10, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(false)
                // lower lift to stack height
                .addTemporalMarker(() -> run_Lift(STACK, lift, liftRear))
                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -(RIGHT_STACK_Y-3), Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(RIGHT_D2_X-0.5, -(RIGHT_D2_Y)), Math.toRadians(225.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                // lift to STACK-1ch
                .addTemporalMarker(() -> run_Lift(STACK - CONE_HEIGHT, lift, liftRear))
                //
                // Cone 1+2
                //
                // return to stack for new cone
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -(RIGHT_STACK_Y-3), Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(RIGHT_D2_X, -(RIGHT_D2_Y)), Math.toRadians(225.00))
                .setReversed(false)
                // open intake
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                // lift to STACK-2ch
                .addTemporalMarker(() -> run_Lift(STACK - (CONE_HEIGHT * 2), lift, liftRear))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -(RIGHT_STACK_Y-3.5), Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(RIGHT_D2_X, -(RIGHT_D2_Y)), Math.toRadians(225.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                .build();

        finalPose = new Pose2d(RIGHT_D2_X, -(RIGHT_D2_Y), Math.toRadians(45.00));

        return trajSeq;
    }

    // B2-B2 Left
    // Code Status: No
    // Comment Status: No
    // Test Status: No
    // TODO: Implement Code
    // TODO: Test Code
    public static TrajectorySequence trajectory_B2_B2(SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {

        // set start position
        Pose2d startPose = new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;
        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around B1
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(90)), Math.toRadians(0))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                //
                // Cone 1
                //
                // drive to D2
                .splineToSplineHeading(new Pose2d(-(RIGHT_B2_X-4), -(RIGHT_B2_Y+10), Math.toRadians(180)), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(-RIGHT_B2_X, -RIGHT_B2_Y, Math.toRadians(180)), Math.toRadians(135))
                // reposition wrist and drop
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)
                // drive around D2

                .setTangent(Math.toRadians(50))
                .splineToSplineHeading(new Pose2d(-16, -14, Math.toRadians(180)), Math.toRadians(180))
                // lower lift to stack height
                .addTemporalMarker(() -> run_Lift(STACK, lift, liftRear))
                //
                // Cone 1+1
                //
                // move to stack for new cone
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X-1), -(LEFT_STACK_Y+1), Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(-(LEFT_B2_X), -(LEFT_B2_Y+4)), Math.toRadians(-45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                // lift to STACK-1ch
                .addTemporalMarker(() -> run_Lift(STACK - CONE_HEIGHT, lift, liftRear))
                //
                // Cone 1+2
                //
                // return to stack for new cone
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X), -(LEFT_STACK_Y+1), Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.5)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK position
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(-(LEFT_B2_X), -(LEFT_B2_Y+4.0)), Math.toRadians(-45.00))

                .setReversed(false)
                // open intake
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                // lift to STACK-2ch
                .addTemporalMarker(() -> run_Lift(STACK - (CONE_HEIGHT * 2), lift, liftRear))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X+1), -(LEFT_STACK_Y+0.5), Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)
                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .setReversed(true)
                .splineTo(new Vector2d(-(LEFT_B2_X), -(LEFT_B2_Y+4.0)), Math.toRadians(-45.00))
                .setReversed(false)
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                .waitSeconds(0.1)

                .build();

        finalPose = new Pose2d(-(LEFT_B2_X), -(LEFT_B2_Y+4), Math.toRadians(135.00));

        return trajSeq;
    }

    // C2-D2 Right
    // Code Status: Yes (:
    // Comment Status: No
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_C2_D2(SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {

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
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                //
                // Cone 1
                //
                // drive to C2
                .splineToSplineHeading(new Pose2d(RIGHT_C2_X+4, -(RIGHT_C2_Y+12), Math.toRadians(180)), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(RIGHT_C2_X, -RIGHT_C2_Y, Math.toRadians(180)), Math.toRadians(180))

                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.1)
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.3)

                // set lift to STACK
                .addTemporalMarker(() -> run_Lift(STACK, lift, liftRear))

                .setReversed(true)
                //
                // CONE 1+1
                //
                // move to stack for new cone
                .setTangent(70)
                .splineToSplineHeading(new Pose2d(24, -12, Math.toRadians(180)), Math.toRadians(0))
                // lower lift to stack height
                // reposition wrist to BACK

                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X + 0.5, -(RIGHT_STACK_Y), Math.toRadians(180)), Math.toRadians(0))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to D3
                .splineTo(new Vector2d(RIGHT_D2_X, -(RIGHT_D2_Y)), Math.toRadians(225.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.2)

                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.2)

                // lift to STACK-1ch
                .addTemporalMarker(() -> run_Lift(STACK - CONE_HEIGHT, lift, liftRear))

                //
                // Cone 1+2
                //
                // return to stack for new cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+1, -(RIGHT_STACK_Y - 0.5), Math.toRadians(180)), Math.toRadians(0))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to D3
                .splineTo(new Vector2d(RIGHT_D2_X, -(RIGHT_D2_Y)), Math.toRadians(225.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.2)

                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.2)

                // lift to STACK-2ch
                .addTemporalMarker(() -> run_Lift(STACK - (CONE_HEIGHT * 2), lift, liftRear))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X+1.5, -(RIGHT_STACK_Y - 0.5), Math.toRadians(180)), Math.toRadians(0))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to D3
                .splineTo(new Vector2d(RIGHT_D2_X, -(RIGHT_D2_Y)), Math.toRadians(225.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.2)

                .build();

        finalPose = new Pose2d((RIGHT_D2_X), -(RIGHT_D2_Y), Math.toRadians(225.00));

        return trajSeq;
    }

    // C2-B2 Left
    // Code Status: Yes (:
    // Comment
    // Test Status: No
    // TODO: Test Code
    public static TrajectorySequence trajectory_C2_B2(SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {

        // set start pose
        Pose2d startPose = new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq;
        // create trajectory sequence
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // drive around B1
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(90)), Math.toRadians(0))
                // lift to HIGH
                .addTemporalMarker(() -> run_Lift(HIGH, lift, liftRear))
                //
                // Cone 1
                //
                // drive to C2
                .splineToSplineHeading(new Pose2d(-(LEFT_C2_X+4), -(LEFT_C2_Y+12), Math.toRadians(0)), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(-LEFT_C2_X, -LEFT_C2_Y, Math.toRadians(0)), Math.toRadians(0))
                // reposition wrist and drop
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.1)
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.3)

                // lift to STACK height
                .addTemporalMarker(() -> run_Lift(STACK, lift, liftRear))

                .setReversed(true)
                //
                // CONE 1+1
                //
                // move to stack for new cone
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(0)), Math.toRadians(180))

                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X), -(LEFT_STACK_Y), Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .splineTo(new Vector2d(-(LEFT_B2_X + 1), -(LEFT_B2_Y + 1)), Math.toRadians(-45.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.1)

                // lift to STACK-1ch
                .addTemporalMarker(() -> run_Lift(STACK - CONE_HEIGHT, lift, liftRear))

                //
                // Cone 1+2
                //
                // return to stack for new cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X+1), -(LEFT_STACK_Y - 1), Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .splineTo(new Vector2d(-(LEFT_B2_X + 1), -(LEFT_B2_Y + 1)), Math.toRadians(-45.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                // reposition wrist to BACK
                .addTemporalMarker(() -> wrist.moveAbsolute(BACK))
                .waitSeconds(0.1)

                // lift to STACK-2ch
                .addTemporalMarker(() -> run_Lift(STACK - (CONE_HEIGHT * 2), lift, liftRear))

                //
                // Cone 1+3
                //
                // return to stack for cone 3
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-(LEFT_STACK_X+1), -(LEFT_STACK_Y - 75), Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .addTemporalMarker(() -> intake.moveAbsolute(CLOSED))
                .waitSeconds(0.3)

                // lift cone off of stack
                .addTemporalMarker(() -> run_Lift(STACK_SAFE, lift, liftRear))
                // reposition wrist to FRONT position
                .addTemporalMarker(() -> wrist.moveAbsolute(FRONT))
                // lift to MEDIUM
                .addTemporalMarker(() -> run_Lift(MEDIUM, lift, liftRear))
                // drive to B2
                .splineTo(new Vector2d(-(LEFT_B2_X + 1), -(LEFT_B2_Y + 1)), Math.toRadians(-45.00))
                // open intake and back up
                .addTemporalMarker(() -> intake.moveAbsolute(OPEN)) // theoretical +10 points
                .waitSeconds(0.3)

                .build();

        finalPose = new Pose2d(-(LEFT_B2_X), -(LEFT_B2_Y), Math.toRadians(-45.00));

        return trajSeq;
    }

    // Parking Left
    // Code Status: Yes (:
    // Test Status: Yes
    public static TrajectorySequence trajectory_ParkLeft(double parkZone, boolean reversed, SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {

        // create trajectory sequence
        TrajectorySequence trajSeq;
        if (reversed) {
            trajSeq = drive.trajectorySequenceBuilder(finalPose)
                    .back(2)
                    // drive to correct zone
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-(36 + parkZone), -13, Math.toRadians(90)), Math.toRadians(0))
                    .setReversed(false)

                    // lift to stack DRIVE
                    .addTemporalMarker(() -> run_Lift(DRIVE, lift, liftRear))
                    .waitSeconds(2)
                    .build();
        } else {
            trajSeq = drive.trajectorySequenceBuilder(finalPose)
                    .forward(2)
                    // drive to correct zone
                    .splineToLinearHeading(new Pose2d(-(36 + parkZone), -13, Math.toRadians(90)), Math.toRadians(0))

                    // lift to stack DRIVE
                    .addTemporalMarker(() -> run_Lift(DRIVE, lift, liftRear))
                    .waitSeconds(2)
                    .build();
        }

        return trajSeq;
    }

    // Parking Right
    // Code Status: Yes (:
    // Test Status: Yes
    public static TrajectorySequence trajectory_ParkRight(double parkZone, boolean reversed, SampleMecanumDrive drive, DcMotor lift, DcMotor liftRear, WristTool wrist, IntakeTool intake) {

        // create trajectory sequence
        TrajectorySequence trajSeq;
        if (reversed) {
            trajSeq = drive.trajectorySequenceBuilder(finalPose)
                    .back(2)
                    // drive to correct zone
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d((36 - parkZone), -12, Math.toRadians(90)), Math.toRadians(0))
                    .setReversed(false)

                    // lift to stack DRIVE
                    .addTemporalMarker(() -> run_Lift(DRIVE, lift, liftRear))
                    .waitSeconds(2)
                    .build();
        } else {
            trajSeq = drive.trajectorySequenceBuilder(finalPose)

                    .forward(2)
                    // drive to correct zone
                    .splineToLinearHeading(new Pose2d((36 - parkZone), -12, Math.toRadians(90)), Math.toRadians(0))

                    // lift to stack DRIVE
                    .addTemporalMarker(() -> run_Lift(DRIVE, lift, liftRear))
                    .waitSeconds(2)
                    .build();
        }

        return trajSeq;
    }

    public static void run_Lift (int liftHeight, DcMotor lift, DcMotor liftRear){

        liftRear.setTargetPosition(liftHeight);
        //liftRear.setTargetPosition(liftHeight);
        liftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRear.setPower(1);
        //liftRear.setPower(1);
    }
}

