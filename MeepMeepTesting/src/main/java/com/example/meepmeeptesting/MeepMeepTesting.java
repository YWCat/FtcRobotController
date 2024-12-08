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
    static double beginX = pos_multiplier*(-24), beginY = pos_multiplier*(-botLengthHalf+72), beginH = 0;
    static double chamberX = pos_multiplier*(-2-botLengthHalf), chamberY = pos_multiplier*(19+botWidthHalf), chamberH = beginH;
    static double firstSample_X = -55*pos_multiplier, Sample_Y = 13*pos_multiplier, Sample_H = 0; //X:38

    public static void main(String[] args) {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(chamberX,chamberY),-Math.PI/2)
                .lineToY(pos_multiplier*(27+3.75))
                .splineToLinearHeading(new Pose2d(-35*pos_multiplier, 36*pos_multiplier, 0), -Math.PI/2)
                .splineToSplineHeading(new Pose2d(-55*pos_multiplier,Sample_Y,0), Math.PI/2)
                .setTangent(-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .setTangent(Math.PI/2)
                .lineToY(12*pos_multiplier)
                .splineToLinearHeading(new Pose2d(-67*pos_multiplier, Sample_Y,0),Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(60*pos_multiplier)
                .setTangent(Math.PI/2)
                .lineToY(12*pos_multiplier)
                .splineToLinearHeading(new Pose2d(-76.5*pos_multiplier, Sample_Y,0),Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(67*pos_multiplier)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}