package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;
    static double beginX = pos_multiplier*(24+botWidthHalf), beginY = pos_multiplier*(-botLengthHalf+72), beginH = Math.PI/2;
    static double basket_X = pos_multiplier*56, basket_Y = pos_multiplier*52, basket_H = Math.PI/4;
    static double sample1_X = pos_multiplier*49, sample_Y = pos_multiplier*(29+botLengthHalf), sample1_H = Math.PI/2;

    public static void main(String[] args) {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d basketPose = new Pose2d(basket_X, basket_Y, basket_H);
        Pose2d sample1Pose = new Pose2d(sample1_X, sample_Y, sample1_H);
        Pose2d sample2Pose = new Pose2d(sample1_X-10, sample_Y, sample1_H);
        Pose2d sample3Pose = new Pose2d((sample1_X-18)+(botLengthHalf*Math.cos(45)), -29-(botLengthHalf*Math.sin(45)), 3*Math.PI/4);

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(beginPose)
                .splineToLinearHeading(basketPose, 3*Math.PI/2)
                .splineToLinearHeading(sample1Pose, Math.PI)
                /*
                .splineToLinearHeading(basketPose, Math.PI/2)
                .splineToLinearHeading(sample2Pose, Math.PI)
                .splineToLinearHeading(basketPose, Math.PI/2)
                .splineToLinearHeading(sample3Pose, Math.PI)
                .splineToLinearHeading(basketPose, Math.PI)

                 */
                //.splineToSplineHeading(new Pose2d(pos_multiplier*35,pos_multiplier*(0),Math.PI/2), -Math.PI/2)
                //.splineToLinearHeading(new Pose2d(pos_multiplier*18,pos_multiplier*(-5),0), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}