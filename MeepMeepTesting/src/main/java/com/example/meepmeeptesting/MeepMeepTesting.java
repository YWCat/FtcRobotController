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
    static double beginX = pos_multiplier*(botWidthHalf), beginY = pos_multiplier*(-botLengthHalf+72), beginH = -Math.PI*pos_multiplier;
    static double chamberX = beginX, chamberY = pos_multiplier*(18+botWidthHalf), chamberH = beginH;
    static double firstSample_X = pos_multiplier*39.5, Sample_Y = pos_multiplier*24, Sample_H = -Math.PI*pos_multiplier; //X:38
    static double basket_X = pos_multiplier*56, basket_Y = pos_multiplier*53, basket_H = Math.PI+(-Math.PI/4*pos_multiplier);

    public static void main(String[] args) {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(beginX, chamberY, beginH);
        Pose2d fstSamplePose = new Pose2d(firstSample_X, Sample_Y, Sample_H);
        Pose2d sndSamplePose = new Pose2d(firstSample_X+(pos_multiplier*10), Sample_Y, Sample_H);
        Pose2d thdSamplePose = new Pose2d(firstSample_X+(pos_multiplier*20), Sample_Y, Sample_H);
        Pose2d basketPose = new Pose2d(basket_X, basket_Y, basket_H);
        Pose2d basket2Pose = new Pose2d(basket_X-pos_multiplier*2, basket_Y-pos_multiplier*4, basket_H);

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(basket2Pose)
                .splineToSplineHeading(new Pose2d(pos_multiplier*38,pos_multiplier*18,Math.PI/2), -Math.PI/2)
                .splineToLinearHeading(new Pose2d(pos_multiplier*25,pos_multiplier*10,0), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}