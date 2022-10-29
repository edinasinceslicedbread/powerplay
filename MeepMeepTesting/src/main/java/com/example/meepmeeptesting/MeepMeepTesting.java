package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(51, 30, Math.toRadians(289.15), Math.toRadians(60), 10.32)
                .setDimensions(16,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -64.5, Math.toRadians(90)))
                                .lineTo(new Vector2d(36,-36))
                                .strafeTo(new Vector2d(12,-36))
                                .lineTo(new Vector2d(12,-12))
                                .strafeTo(new Vector2d(36,-12))
                                .lineTo(new Vector2d(36,12))
                                .strafeTo(new Vector2d(12, 12))
                                .lineTo(new Vector2d(12, 36))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}