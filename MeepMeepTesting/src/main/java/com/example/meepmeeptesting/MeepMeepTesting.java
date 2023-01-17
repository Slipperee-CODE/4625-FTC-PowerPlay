package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.ArrayList;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);





        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(38, -61.5, Math.toRadians(90)))


                                .strafeLeft(4.5)
                                .forward(50)


                                .strafeLeft(10)
                                .forward(5)
                                .back(5)
                                .strafeRight(12.5)


                                .turn(Math.toRadians(-90))
                                .forward(30)
                                .back(30)
                                .turn(Math.toRadians(90))


                                .strafeLeft(12.5)
                                .forward(5)
                                .back(5)
                                .strafeRight(12.5)


                                .turn(Math.toRadians(-90)) //Parks in the Center


                                //Alternate Parkings

                                // Parks on the Left
                                //.back(23)

                                // Parks on the Right
                                .forward(23)



                                //.splineTo(new Vector2d(30, 30), 0)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}