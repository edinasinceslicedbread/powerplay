package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
/*
        // Red Right
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32, -64, Math.toRadians(0)))
                                .strafeTo(new Vector2d(33,-12))
                                .lineToLinearHeading(new Pose2d(32, -8, Math.toRadians(-45)))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .back(1)
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .back(1)
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .back(1)
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .back(1)
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(.5)
                                .addTemporalMarker(() -> {})
                                .lineToLinearHeading(new Pose2d(36,-12, Math.toRadians(-90)))
                                .lineTo(new Vector2d(15,-10))
                                .build()
                );
*/

        // Red Right
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32, -64, Math.toRadians(90)))
                                .lineTo(new Vector2d(14, -60))
                                .splineToLinearHeading(new Pose2d(10,-24, Math.toRadians(180)), Math.toRadians(90))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(1)
                                .addTemporalMarker(() -> {})
                                //.lineToLinearHeading(new Pose2d(14, -24, Math.toRadians(90)))
                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(1)
                                .addTemporalMarker(() -> {})
                                .back(1)
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(1)
                                .addTemporalMarker(() -> {})
                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(1)
                                .addTemporalMarker(() -> {})
                                .back(1)
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(1)
                                .addTemporalMarker(() -> {})
                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(1)
                                .addTemporalMarker(() -> {})
                                .back(1)
                                .splineTo(new Vector2d(32, -8), Math.toRadians(135.00))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(1)
                                .addTemporalMarker(() -> {})
                                .lineToLinearHeading(new Pose2d(36,-12, Math.toRadians(90)))
                                .lineTo(new Vector2d(12,-12))
                .build()
                );

        // Blue Left
/*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(300), Math.toRadians(60), 8)
                .setDimensions(14,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(42, 64, Math.toRadians(-90)))
                                .lineTo(new Vector2d(12, 58))
                                .splineToLinearHeading(new Pose2d(7,31, Math.toRadians(-135)), Math.toRadians(-135))
                                .addTemporalMarker(() -> {})
                                .waitSeconds(2)
                                .addTemporalMarker(() -> {})
                                .lineToSplineHeading(new Pose2d(12,36, Math.toRadians(-90)))
                                .strafeRight(0-48.99) // flip it!
                                .build()
                );
*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}