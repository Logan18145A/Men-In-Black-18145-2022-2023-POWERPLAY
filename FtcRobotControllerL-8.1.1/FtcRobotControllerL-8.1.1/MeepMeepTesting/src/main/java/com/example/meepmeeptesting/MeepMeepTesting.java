package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose           = new Pose2d(-37, 63, Math.toRadians(90));
        //Based off the direction you setup
        Pose2d RightBlueStack      = new Pose2d(-60, 12, Math.toRadians(0));
        Pose2d LeftBlueStack       = new Pose2d(60, 12, Math.toRadians(0));
        Pose2d RightRedStack       = new Pose2d(60, -12, Math.toRadians(0));
        Pose2d LeftRedStack        = new Pose2d(-60, -12, Math.toRadians(0));
        //Vectors for Consistency
        Vector2d RightBlueStackV      = new Vector2d(-60,12);
        Vector2d LeftBlueStackV       = new Vector2d(60,12);
        Vector2d RightRedStackV       = new Vector2d(60,-12);
        Vector2d LeftRedStackV        = new Vector2d(-60,-12);

        //Poles
        Pose2d PreLoadMiddleHigh   = new Pose2d(10.5,   -23,Math.toRadians(90));
        Pose2d RightHighPole       = new Pose2d(24,-12,Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(180), Math.toRadians(180), 11)
                .followTrajectorySequence(drive ->
                             // The following is the swampbots 1+4 auto for a medium cycle, I am attempting a high to medium cycle which is exihibited below
                               //Pre-Load
                        drive.trajectorySequenceBuilder(new Pose2d(34.97, -60.52, Math.toRadians(90.00)))
                                .forward(20)
                                .splineTo(new Vector2d(PreLoadMiddleHigh.getX(), PreLoadMiddleHigh.getY()), PreLoadMiddleHigh.getHeading())
                                .waitSeconds(.5)
                                 //Cone 1
                                .splineTo(new Vector2d(RightRedStack.getX(), RightRedStack.getY()), RightRedStack.getHeading())
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(RightHighPole.getX(), RightHighPole.getY()), RightHighPole.getHeading())
                                .waitSeconds(0.25)
                                //Cone 2
                                .lineTo(RightRedStackV)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(RightHighPole.getX(), RightHighPole.getY()), RightHighPole.getHeading())
                                .waitSeconds(.25)
                                //Cone 3
                                .lineTo(RightRedStackV)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(RightHighPole.getX(), RightHighPole.getY()), RightHighPole.getHeading())
                                .waitSeconds(.25)

                                //Cone 4
                                .lineTo(RightRedStackV)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(RightHighPole.getX(), RightHighPole.getY()), RightHighPole.getHeading())
                                .waitSeconds(.25)
                                //Cone 5
                                .lineTo(RightRedStackV)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(RightHighPole.getX(), RightHighPole.getY()), RightHighPole.getHeading())
                                .waitSeconds(.25)
                                //parking spot 1
                                // .back(12)
                                //parking spot 2
                                // .forward(12)
                                //parking spot 3
                                .forward(36)
                                .build()
                );






        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}