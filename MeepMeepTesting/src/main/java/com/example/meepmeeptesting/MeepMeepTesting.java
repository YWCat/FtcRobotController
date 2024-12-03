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
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(pos_multiplier*-botLengthHalf, pos_multiplier*62.5, 0))
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(pos_multiplier*(2-botLengthHalf),pos_multiplier*(18+botWidthHalf)),Math.PI/2)
                .lineToY(pos_multiplier*(27+3.75))
                .splineToLinearHeading(new Pose2d(-32*pos_multiplier, 36*pos_multiplier, 0), Math.PI/2)
                .splineToSplineHeading(new Pose2d(-48*pos_multiplier,16*pos_multiplier,0), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .setTangent(Math.PI/2)
                .lineToY(12*pos_multiplier)
                .splineToLinearHeading(new Pose2d(-58*pos_multiplier, 16*pos_multiplier,0),-Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(64*pos_multiplier)
                .setTangent(Math.PI/2)
                .lineToY(12*pos_multiplier)
                .splineToLinearHeading(new Pose2d(-63*pos_multiplier, 16*pos_multiplier,0),-Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(64*pos_multiplier)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}