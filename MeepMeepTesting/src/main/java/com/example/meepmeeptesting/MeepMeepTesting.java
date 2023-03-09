package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        // sab: mai trebuie lucrat
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .lineToConstantHeading(new Vector2d(0,2))
                                .lineToLinearHeading(new Pose2d(6, 15, Math.toRadians(45)))
                                .addDisplacementMarker(40, () -> {
                                    ;
//                                    lift.moveLift(4030);
                                })
                                .lineToConstantHeading(new Vector2d(6, 55))
                                .lineToConstantHeading(new Vector2d(12, 57))
                                .lineToConstantHeading(new Vector2d(14, 59.3))
                                .waitSeconds(4)
                                .addSpatialMarker(new Vector2d(14, 59.3), () -> {
                                    ;
//                                    servo_gheara.setPosition(0.13);
                                })
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(12.1, 57.8))
                                .waitSeconds(2)
                                .addDisplacementMarker(() -> {
                                    ;
//                                    lift.moveLift(500);
                                })
                                .waitSeconds(4)
//                                .lineToConstantHeading(new Vector2d(6, 56))
//                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-10, 58, Math.toRadians(180)))
                                .waitSeconds(3)
                                .build()
                );
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(6, 55, Math.toRadians(45)))
//                            .lineToConstantHeading(new Vector2d(16, 62))
//                            .build()
//                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}