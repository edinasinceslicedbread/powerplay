package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // Starting points
    final static double START_LEFT_X = 41.75;
    final static double START_LEFT_Y = 64;
    final static double START_RIGHT_X = 32;
    final static double START_RIGHT_Y = 64;

    // coordinate autonomous constants
    final static double B3_X = 34;
    final static double B3_Y = 6;

    final static double D3_X = 32;
    final static double D3_Y = 5.5;

    final static double LEFT_D2_X = 15;
    final static double LEFT_D2_Y = 24; // check for accuracy

    final static double RIGHT_D2_X = 32;
    final static double RIGHT_D2_Y = 18.5; // check for accuracy

    final static double RIGHT_B2_X = 15.5;
    final static double RIGHT_B2_Y = 23; // check for accuracy
    final static double LEFT_B2_X = 35;
    final static double LEFT_B2_Y = 19.5; // check for accuracy

    final static double RIGHT_C2_X = 8.5;
    final static double RIGHT_C2_Y = 23;
    final static double LEFT_C2_X = 9;
    final static double LEFT_C2_Y = 26;

    // Cone stack coordinates
    final static double RIGHT_STACK_X = 64;
    final static double RIGHT_STACK_Y = 12;
    final static double LEFT_STACK_X = 64;
    final static double LEFT_STACK_Y = 12.5;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot;

        //myBot = trajectory_C2_D3(meepMeep);
        //myBot = trajectory_C2_B3(meepMeep);
        //myBot = trajectory_D2_D3(meepMeep);
        //myBot = trajectory_B2_B3(meepMeep);
        //myBot = trajectory_C2_D2(meepMeep);
        //myBot = trajectory_C2_B2(meepMeep);
        //myBot = trajectory_D2_D2(meepMeep);
        myBot = trajectory_B2_B2(meepMeep);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    // RIGHT C2-D3
    // Updated to fast spline 1/21
    // Implemented - yes
    public static RoadRunnerBotEntity trajectory_C2_D3(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
            .setDimensions(12.5, 16)
            .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90)))
                        // reposition wrist to front
                        // drive around D1
                        .setTangent(-180)
                        .splineToSplineHeading(new Pose2d(20, -57, Math.toRadians(90)), Math.toRadians(135))
                        // raise lift to HIGH
                        // drive to C2

                            .splineToSplineHeading(new Pose2d(RIGHT_C2_X+4, -(RIGHT_C2_Y+12), Math.toRadians(180)), Math.toRadians(90))

                            .splineToSplineHeading(new Pose2d(RIGHT_C2_X, -RIGHT_C2_Y, Math.toRadians(180)), Math.toRadians(180))
                        .waitSeconds(1)
                        // back away from C2

                        .setReversed(true)
                        .setTangent(70)
                        .splineToSplineHeading(new Pose2d(22, -12, Math.toRadians(180)), Math.toRadians(0))
                        // lower lift to stack height
                        // move to stack for new cone

                        .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                        // open intake and back up
                        .waitSeconds(0.5)
                        // return to stack for new cone
                        .setReversed(true)
                        .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                        // open intake and back up
                        .waitSeconds(0.5)
                        // return to stack for new cone
                        .setReversed(true)
                        .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                        // open intake and back up
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                        // open intake and back up
                        .waitSeconds(0.5)
                            .back(2)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(12, -12, Math.toRadians(90)), Math.toRadians(0))
                        .setReversed(false)
                        //.strafeLeft(-23.99)
                        .build()
            );
        return myBot;
    }

    // LEFT C2-B3
    // Updated to fast spline 1/24
    // Implemented -
    public static RoadRunnerBotEntity trajectory_C2_B3(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
            .setDimensions(12.5,16)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90)))
                    // reposition wrist to front
                    // drive around D1
                    .setTangent(45)
                    .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(90)), Math.toRadians(0))
                    // raise lift to HIGH
                    // drive to C2
                    .splineToSplineHeading(new Pose2d(-(LEFT_C2_X+1), -LEFT_C2_Y, Math.toRadians(0)), Math.toRadians(90))

                    .waitSeconds(0.5)
                    // back away from C2
                    .setReversed(true)
                    .setTangent(90)
                    .splineToSplineHeading(new Pose2d(-18, -12, Math.toRadians(0)), Math.toRadians(180))
                    // lower lift to stack height
                    // move to stack for new cone

                    .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(1)

                    // reposition wrist
                    // drive to B3
                    .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    // return to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(0.5)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    // return to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(0.5)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    .setReversed(true)
                    .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(0.5)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)), Math.toRadians(45))
                    .setReversed(false)
                    //.strafeLeft(-23.99)
                    .build()
            );
        return myBot;
    }

    // RIGHT D2-D3
    // Updated to fast spline 1/27
    // Implemented -
    public static RoadRunnerBotEntity trajectory_D2_D3(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(12.5, 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .setTangent(-180)
                                .splineToSplineHeading(new Pose2d(14, -54, Math.toRadians(90)), Math.toRadians(135))
                                // raise lift to HIGH
                                // drive to D2
                                .splineToSplineHeading(new Pose2d((LEFT_D2_X-4), -(LEFT_D2_Y+10), Math.toRadians(0)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(LEFT_D2_X, -LEFT_D2_Y, Math.toRadians(0)), Math.toRadians(45))

                                //.splineToSplineHeading(new Pose2d(12, -30, Math.toRadians(180)), Math.toRadians(90))
                                //.splineToSplineHeading(new Pose2d(LEFT_D2_X, -LEFT_D2_Y, Math.toRadians(180)), Math.toRadians(45))

                                .waitSeconds(1)
                                // back away from D2

                                .setTangent(Math.toRadians(110))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(12, -10, Math.toRadians(0)), Math.toRadians(0))
                                .setReversed(false)
                                // lower lift to stack height
                                // move to stack for new cone

                                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))

                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(D3_X, -D3_Y), Math.toRadians(135.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(12, -12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)
                                //.strafeLeft(-23.99)
                                .build()
                );
        return myBot;
    }

    // LEFT B2-B3
    // Fast Spline - 1/27
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_B2_B3(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
            .setDimensions(12.5,16)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90)))                                // reposition wrist to front
                        .setTangent(45)
                        .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(90)), Math.toRadians(0))
                        // raise lift to HIGH
                        // drive to D2
                        .splineToSplineHeading(new Pose2d(-12, -30, Math.toRadians(180)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-RIGHT_B2_X, -RIGHT_B2_Y, Math.toRadians(180)), Math.toRadians(135))
                        .waitSeconds(1)
                        // back away from D2

                        .setTangent(Math.toRadians(70))
                        .splineToSplineHeading(new Pose2d(-20, -12, Math.toRadians(180)), Math.toRadians(180))
                        // lower lift to stack height
                        // move to stack for new cone

                        .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(180)), Math.toRadians(180))
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .setReversed(true)
                        .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                        .setReversed(false)
                        // open intake and back up
                        .waitSeconds(0.5)
                        // return to stack for new cone
                        .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .setReversed(true)
                        .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                        .setReversed(false)
                        // open intake and back up
                        .waitSeconds(0.5)
                        // return to stack for new cone
                        .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to B3
                        .setReversed(true)
                        .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                        .setReversed(false)
                        // open intake and back up
                        .waitSeconds(0.5)
                        .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to B3
                        .setReversed(true)
                        .splineTo(new Vector2d(-B3_X, -B3_Y), Math.toRadians(45.00))
                        .setReversed(false)
                        // open intake and back up
                        .waitSeconds(0.5)

                        .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)), Math.toRadians(0))

                        //.strafeLeft(-23.99)
                        .build()
            );
        return myBot;
    }

    // RIGHT C2-D2
    // Fast Spline - 1/27
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_C2_D2(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(12.5, 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .setTangent(-180)
                                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
                                // raise lift to HIGH
                                // drive to C2
                                .splineToSplineHeading(new Pose2d(RIGHT_C2_X, -RIGHT_C2_Y, Math.toRadians(180)), Math.toRadians(90))
                                .waitSeconds(1)
                                // back away from C2

                                .setReversed(true)
                                .setTangent(Math.toRadians(70))
                                .splineToSplineHeading(new Pose2d(18, -10, Math.toRadians(180)), Math.toRadians(0))
                                // lower lift to stack height
                                // move to stack for new cone

                                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(12, -12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)
                                //.strafeLeft(-23.99)
                                .build()
                );
        return myBot;
    }

    // LEFT C2-B2
    // Fast Spline - 1/27
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_C2_B2(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(12.5,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-START_LEFT_X, -START_RIGHT_Y, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1

                                .setTangent(45)
                                .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(90)), Math.toRadians(0))
                                // raise lift to HIGH
                                // drive to C2
                                .splineToSplineHeading(new Pose2d(-LEFT_C2_X, -LEFT_C2_Y, Math.toRadians(0)), Math.toRadians(90))
                                .waitSeconds(1)

                                // back away from C2
                                .setReversed(true)
                                .setTangent(90)
                                .splineToSplineHeading(new Pose2d(-18, -12, Math.toRadians(0)), Math.toRadians(180))
                                // lower lift to stack height
                                // move to stack for new cone
                                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(0)), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)

                                // reposition wrist
                                // drive to B2
                                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)
                                //.strafeLeft(-23.99)
                                .build()
                );
        return myBot;
    }

    // RIGHT D2-D2
    // Fast Spline - 1/29
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_D2_D2(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(12.5, 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(START_RIGHT_X, -START_RIGHT_Y, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .setTangent(-180)
                                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
                                // raise lift to HIGH
                                // drive to D2
                                .splineToSplineHeading(new Pose2d(12, -30, Math.toRadians(180)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(LEFT_D2_X, -LEFT_D2_Y, Math.toRadians(180)), Math.toRadians(45))

                                .waitSeconds(1)
                                // back away from D2

                                .setReversed(true)
                                .setTangent(Math.toRadians(110))
                                .splineToSplineHeading(new Pose2d(18, -10, Math.toRadians(180)), Math.toRadians(0))
                                // lower lift to stack height

                                // move to stack for new cone
                                .splineToSplineHeading(new Pose2d(RIGHT_STACK_X, -RIGHT_STACK_Y, Math.toRadians(180)), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D2
                                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to RIGHT STACK for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D2
                                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to RIGHT STACK for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D2
                                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to RIGHT STACK for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(RIGHT_STACK_X, -RIGHT_STACK_Y), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D2
                                .splineTo(new Vector2d(RIGHT_D2_X, -RIGHT_D2_Y), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)

                                // Park
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(12, -12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)

                                .build()
                );
        return myBot;
    }

    // RIGHT B2-B2
    // Fast Spline - 1/29
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_B2_B2(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(12.5,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-START_LEFT_X, -START_LEFT_Y, Math.toRadians(90)))                                // reposition wrist to front
                                // INTERMEDIATE - Go around pole at B1
                                .setTangent(45)
                                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(90)), Math.toRadians(0))
                                // raise lift to HIGH
                                // drive to D2
                                .splineToSplineHeading(new Pose2d(-12, -30, Math.toRadians(180)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-RIGHT_B2_X, -RIGHT_B2_Y, Math.toRadians(180)), Math.toRadians(135))
                                .waitSeconds(1)
                                // back away from D2

                                .setTangent(Math.toRadians(50))
                                .splineToSplineHeading(new Pose2d(-16, -12, Math.toRadians(180)), Math.toRadians(180))
                                // Drive to LEFT STACK
                                .splineToSplineHeading(new Pose2d(-LEFT_STACK_X, -LEFT_STACK_Y, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)

                                // drive to B2
                                .setReversed(true)
                                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                                .setReversed(false)
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to B2
                                .setReversed(true)
                                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                                .setReversed(false)
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                                // Grab cone
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to B2
                                .setReversed(true)
                                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                                .setReversed(false)
                                // open intake
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .splineTo(new Vector2d(-LEFT_STACK_X, -LEFT_STACK_Y), Math.toRadians(180))
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to B2
                                .setReversed(true)
                                .splineTo(new Vector2d(-LEFT_B2_X, -LEFT_B2_Y), Math.toRadians(-45.00))
                                .setReversed(false)
                                // open intake and back up
                                .waitSeconds(0.5)

                                // Park
                                .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)), Math.toRadians(0))

                                .build()
                );
        return myBot;
    }
}

