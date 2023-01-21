package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // RIGHT C2 D3
        RoadRunnerBotEntity myBotRightC2D3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32, -64, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .splineToSplineHeading(new Pose2d(17, -57, Math.toRadians(90)), Math.toRadians(135))
                                // raise lift to HIGH
                                // drive to C2
                                .splineToSplineHeading(new Pose2d(9.0, -24, Math.toRadians(180)), Math.toRadians(90))
                                //.waitSeconds(1)
                                // back away from C2

                                .setReversed(true)
                                //.lineToLinearHeading(new Pose2d(14, -12, Math.toRadians(180)))
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
                                .splineToLinearHeading(new Pose2d(60,-12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)
                                //.strafeLeft(-23.99)
                                .build()
                );

        // LEFT C2 D3
        RoadRunnerBotEntity myBotLeftC2D3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .lineTo(new Vector2d(-14, -60))
                                // raise lift to HIGH
                                // drive to C2
                                .splineToLinearHeading(new Pose2d(-9.0, -24, Math.toRadians(0)), Math.toRadians(90))
                                .waitSeconds(1)
                                // back away from C2
                                .lineToLinearHeading(new Pose2d(-14, -12, Math.toRadians(0)))
                                // lower lift to stack height
                                .setReversed(true)
                                // move to stack for new cone
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to B3
                                .splineToLinearHeading(new Pose2d(-32, -8, Math.toRadians(45)), Math.toRadians(45))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // return to stack for new cone
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineToLinearHeading(new Pose2d(-32, -8, Math.toRadians(45)), Math.toRadians(45))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // return to stack for new cone
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                .splineToLinearHeading(new Pose2d(-32, -8, Math.toRadians(45)), Math.toRadians(45))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-57,-12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)
                                //.strafeLeft(0-23.99)
                                .build()
                );

        // RIGHT D2
        RoadRunnerBotEntity myBotRightD2D3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32, -64, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .splineToLinearHeading(new Pose2d(12, -55, Math.toRadians(180)), Math.toRadians(90))
                                // raise lift to HIGH
                                // drive to C2
                                .strafeTo(new Vector2d(15,-24))
                                .waitSeconds(1)
                                // back away from C2

                                .lineToLinearHeading(new Pose2d(15, -14, Math.toRadians(180)))
                                // lower lift to stack height
                                // move to stack for new cone
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
                                .splineToLinearHeading(new Pose2d(60,-12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)
                                //.strafeLeft(-23.99)
                                .build()
                );

        // LEFT D2-D3
        RoadRunnerBotEntity myBotLeftD2D3 = new DefaultBotBuilder(meepMeep)
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

        // RIGHT C2 D2
        RoadRunnerBotEntity myBotRightC2D2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32, -64, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .lineTo(new Vector2d(14, -60))
                                // raise lift to HIGH
                                // drive to C2
                                .splineToLinearHeading(new Pose2d(9.0, -24, Math.toRadians(180)), Math.toRadians(90))
                                .waitSeconds(1)
                                // back away from C2

                                .lineToLinearHeading(new Pose2d(14, -12, Math.toRadians(180)))
                                // lower lift to stack height
                                // move to stack for new cone
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
                                .splineToLinearHeading(new Pose2d(12,-12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)
                                //.strafeLeft(-23.99)
                                .build()
                );

        // LEFT C2 D2
        RoadRunnerBotEntity myBotLeftC2D2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                                // reposition wrist to front
                                // drive around D1
                                .lineTo(new Vector2d(-14, -60))
                                // raise lift to HIGH
                                // drive to C2
                                .splineToLinearHeading(new Pose2d(-9.0, -24, Math.toRadians(0)), Math.toRadians(90))
                                .waitSeconds(1)
                                // back away from C2
                                .lineToLinearHeading(new Pose2d(-14, -12, Math.toRadians(0)))
                                // lower lift to stack height
                                .setReversed(true)
                                // move to stack for new cone
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to B3
                                //.splineToLinearHeading(new Pose2d(-32, -16, Math.toRadians(-45)), Math.toRadians(0))
                                .splineTo(new Vector2d(-32, -16), Math.toRadians(-45))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // return to stack for new cone
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D2
                                //.splineToLinearHeading(new Pose2d(-32, -16, Math.toRadians(-45)), Math.toRadians(0))
                                .splineTo(new Vector2d(-32, -16), Math.toRadians(-45))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // return to stack for new cone
                                .splineTo(new Vector2d(-58, -11), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                // reposition wrist
                                // drive to D3
                                //.splineToLinearHeading(new Pose2d(-32, -16, Math.toRadians(-45)), Math.toRadians(0))
                                .splineTo(new Vector2d(-32, -16), Math.toRadians(-45))
                                // open intake and back up
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-57,-12, Math.toRadians(90)), Math.toRadians(0))
                                .setReversed(false)
                                //.strafeLeft(0-23.99)
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotRightC2D3)
                .start();
    }
}