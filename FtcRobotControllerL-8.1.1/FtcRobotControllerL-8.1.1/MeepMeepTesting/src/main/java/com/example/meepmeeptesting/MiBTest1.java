package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MiBTest1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose           = new Pose2d(-37, 63, Math.toRadians(90));
        Pose2d PreLoadMidJunction  = new Pose2d(-33, 18, Math.toRadians(45));
        Pose2d Stack               = new Pose2d(-68, 14, Math.toRadians(180));
        Pose2d Middle_Tile         = new Pose2d(-40, 18, Math.toRadians(180));
        Pose2d closeMidJunction    = new Pose2d(-33, 18, Math.toRadians(52));
        Pose2d PreLoadMiddleHigh   = new Pose2d(0,   -24,Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(205.3399090909091), Math.toRadians(205.3399090909091), 11)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(new Pose2d(34.97, -60.52, Math.toRadians(90.00)))
                                .forward(45)
                                .splineTo(new Vector2d(PreLoadMidJunction.getX(), PreLoadMidJunction.getY()), PreLoadMidJunction.getHeading())
                                .build());



        // );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
//  drive.trajectorySequenceBuilder(new Pose2d(-37, 63, Math.toRadians(90)))
                             /* The following is the swampbots 1+4 auto for a medium cycle, I am attempting a high to medium cycle which is exihibited below
                               //Pre-Load
                                .back(45)
                                .splineTo(new Vector2d(PreLoadMidJunction.getX(), PreLoadMidJunction.getY()), PreLoadMidJunction.getHeading())
                                .waitSeconds(0.5)
                                //Stack 1
                                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading())
                                .waitSeconds(0.25)
                                .setReversed(false)
                                //Stack 2
                                .splineTo(new Vector2d(Stack.getX()+1, Stack.getY()), Stack.getHeading())
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading()-1)
                                .waitSeconds(0.25)
                                .setReversed(false)
                                //Stack 3
                                .splineTo(new Vector2d(Stack.getX()+2, Stack.getY()), Stack.getHeading())
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading()-1)
                                .waitSeconds(0.25)
                                .setReversed(false)
                                //Stack 4
                                .splineTo(new Vector2d(Stack.getX()+3, Stack.getY()), Stack.getHeading())
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading()-1)
                                .waitSeconds(0.25)
                                .setReversed(false)
                                .splineTo(new Vector2d(Middle_Tile.getX(), Middle_Tile.getY()), Middle_Tile.getHeading())
                                .build() */