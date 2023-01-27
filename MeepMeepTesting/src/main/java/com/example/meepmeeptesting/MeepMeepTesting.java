package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot;

        myBot = trajectory_D2_D3_right(meepMeep);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    // RIGHT C2-D3
    // Updated to fast spline 1/21
    // Implemented - yes
    public static RoadRunnerBotEntity trajectory_C2_D3_right(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
            .setDimensions(14, 15)
            .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(32, -64, Math.toRadians(90)))
                        // reposition wrist to front
                        // drive around D1
                        .setTangent(-180)
                        .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
                        // raise lift to HIGH
                        // drive to C2
                        .splineToSplineHeading(new Pose2d(9.0, -24, Math.toRadians(180)), Math.toRadians(90))
                        .waitSeconds(1)
                        // back away from C2

                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(22, -12, Math.toRadians(180)), Math.toRadians(0))
                        // lower lift to stack height
                        // move to stack for new cone

                        .splineToSplineHeading(new Pose2d(58, -11, Math.toRadians(180)), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                        // open intake and back up
                        .waitSeconds(0.5)
                        // return to stack for new cone
                        .setReversed(true)
                        .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                        // open intake and back up
                        .waitSeconds(0.5)
                        // return to stack for new cone
                        .setReversed(true)
                        .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                        // open intake and back up
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(1)
                        // reposition wrist
                        // drive to D3
                        .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
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

    // LEFT C2-B3
    // Updated to fast spline 1/24
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_C2_B3_left(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
            .setDimensions(14,15)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                    // reposition wrist to front
                    // drive around D1
                    .setTangent(0)
                    .splineToSplineHeading(new Pose2d(-18, -59, Math.toRadians(90)), Math.toRadians(45))
                    // raise lift to HIGH
                    // drive to C2
                    .splineToSplineHeading(new Pose2d(-9.0, -24, Math.toRadians(0)), Math.toRadians(90))
                    .waitSeconds(1)

                    // back away from C2
                    .setReversed(true)
                    .setTangent(90)
                    .splineToSplineHeading(new Pose2d(-18, -12, Math.toRadians(0)), Math.toRadians(180))
                    // lower lift to stack height
                    // move to stack for new cone

                    .splineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(0)), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(1)

                    // reposition wrist
                    // drive to B3
                    .splineTo(new Vector2d(-32, -8), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    // return to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(1)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-32, -8), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    // return to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(1)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-32, -8), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    .setReversed(true)
                    .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(1)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-32, -8), Math.toRadians(45.00))
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

    // RIGHT D2-D3
    //
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_D2_D3_right(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32, -64, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .setTangent(-180)
                                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
                                // raise lift to HIGH
                                // drive to D2
                                .splineToSplineHeading(new Pose2d(15, -30, Math.toRadians(180)), Math.toRadians(90))

                                .splineToSplineHeading(new Pose2d(15, -24, Math.toRadians(180)), Math.toRadians(90))
                                .waitSeconds(1)
                                // back away from D2

                                .setReversed(true)
                                .setTangent(Math.toRadians(110))
                                .splineToSplineHeading(new Pose2d(18, -10, Math.toRadians(180)), Math.toRadians(0))
                                // lower lift to stack height
                                // move to stack for new cone

                                .splineToSplineHeading(new Pose2d(58, -11, Math.toRadians(180)), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
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

    // LEFT D2-B3
    //
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_B2_B3_left(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
            .setDimensions(14,15)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))                                // reposition wrist to front
                    // drive around D1
                    .splineToLinearHeading(new Pose2d(-12, -55, Math.toRadians(0)), Math.toRadians(90))
                    // raise lift to HIGH
                    // drive to C2
                    .strafeTo(new Vector2d(-15,-24))
                    .waitSeconds(1)
                    // back away from C2

                    .lineToLinearHeading(new Pose2d(-15, -14, Math.toRadians(0)))
                    // lower lift to stack height
                    // move to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(1)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-32, -8), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    // return to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(1)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-32, -8), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    // return to stack for new cone
                    .setReversed(true)
                    .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                    .setReversed(false)
                    .waitSeconds(1)
                    // reposition wrist
                    // drive to D3
                    .splineTo(new Vector2d(-32, -8), Math.toRadians(45.00))
                    // open intake and back up
                    .waitSeconds(0.5)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-60,-12, Math.toRadians(90)), Math.toRadians(0))
                    .setReversed(false)
                    //.strafeLeft(-23.99)
                    .build()
            );
        return myBot;
    }

    // RIGHT C2-D2
    //
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_C2_D2_right(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32, -64, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .setTangent(-180)
                                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
                                // raise lift to HIGH
                                // drive to C2
                                .splineToSplineHeading(new Pose2d(9.0, -24, Math.toRadians(180)), Math.toRadians(90))
                                .waitSeconds(1)
                                // back away from C2

                                .setReversed(true)
                                .setTangent(Math.toRadians(70))
                                .splineToSplineHeading(new Pose2d(18, -10, Math.toRadians(180)), Math.toRadians(0))
                                // lower lift to stack height
                                // move to stack for new cone

                                .splineToSplineHeading(new Pose2d(58, -11, Math.toRadians(180)), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(32, -16), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(32, -16), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(32, -16), Math.toRadians(225.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(58, -11), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(32, -16), Math.toRadians(225.00))
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
    //
    // Implemented - no
    public static RoadRunnerBotEntity trajectory_C2_B2_left(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .splineToSplineHeading(new Pose2d(-18, -59, Math.toRadians(90)), Math.toRadians(45))
                                // raise lift to HIGH
                                // drive to C2
                                .splineToSplineHeading(new Pose2d(-9.0, -24, Math.toRadians(0)), Math.toRadians(90))
                                .waitSeconds(1)

                                // back away from C2
                                .setReversed(true)
                                .setTangent(90)
                                .splineToSplineHeading(new Pose2d(-18, -12, Math.toRadians(0)), Math.toRadians(180))
                                // lower lift to stack height
                                // move to stack for new cone
                                .splineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(0)), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)

                                // reposition wrist
                                // drive to B2
                                .splineTo(new Vector2d(-32, -16), Math.toRadians(-45.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(-32, -16), Math.toRadians(-45.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                // return to stack for new cone
                                .setReversed(true)
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(-32, -16), Math.toRadians(-45.00))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineTo(new Vector2d(-32, -16), Math.toRadians(-45.00))
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
    // NOTE: stub only
    public static RoadRunnerBotEntity trajectory_D2_D2_right(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                                .build()
                );
        return myBot;
    }

    // RIGHT B2-B2
    // NOTE: stub only
    public static RoadRunnerBotEntity trajectory_B2_B2_left(MeepMeep meepMeep) {

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                                .build()
                );
        return myBot;
    }
}

