package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static int pos_multiplier = 1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;
    static double beginX = pos_multiplier*(botWidthHalf), beginY = pos_multiplier*(-botLengthHalf+72), beginH = 0;
    static double chamberX = beginX, chamberY = pos_multiplier*(18+botWidthHalf), chamberH = beginH;
    static double firstSample_X = pos_multiplier*39.5, Sample_Y = pos_multiplier*24, Sample_H = 0; //X:38
    static double basket_X = pos_multiplier*56, basket_Y = pos_multiplier*52, basket_H = (3*Math.PI/4)+(-Math.PI/2*pos_multiplier);

    public static void main(String[] args) {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(beginX, chamberY, beginH);
        Pose2d fstSamplePose = new Pose2d(firstSample_X, Sample_Y, Sample_H);
        Pose2d sndSamplePose = new Pose2d(firstSample_X+(pos_multiplier*10), Sample_Y, Sample_H);
        Pose2d thdSamplePose = new Pose2d(firstSample_X+(pos_multiplier*20), Sample_Y, Sample_H);
        Pose2d basketPose = new Pose2d(basket_X, basket_Y, basket_H);
        Pose2d basket2Pose = new Pose2d(basket_X-pos_multiplier*2, basket_Y-pos_multiplier*4, basket_H);
        Pose2d basketBackPose = new Pose2d(pos_multiplier*48,pos_multiplier*48,0);
        Pose2d basketBack2Pose = new Pose2d(pos_multiplier*42, pos_multiplier*42, 0);

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY)
                .lineToY(chamberY+(pos_multiplier*7))
                .splineToLinearHeading(fstSamplePose, pos_multiplier*-Math.PI/2)
                .splineToLinearHeading(basketPose, 0)


                .splineToLinearHeading(basketBackPose, Math.PI)
                .splineToLinearHeading(basketBack2Pose, 0)
                .splineToLinearHeading(sndSamplePose, 0)
                .splineToLinearHeading(basket2Pose, 0)
                .splineToSplineHeading(new Pose2d(pos_multiplier*35,pos_multiplier*(0),Math.PI/2), Math.PI/2)
                .splineToLinearHeading(new Pose2d(pos_multiplier*18,pos_multiplier*(-5),0), Math.PI)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}