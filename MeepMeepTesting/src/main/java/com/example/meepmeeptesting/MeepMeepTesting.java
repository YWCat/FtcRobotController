package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static int pos_multiplier = -1;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(pos_multiplier*(6.085+0), pos_multiplier*(-3.75+72), Math.PI))
                //Deposit Preload
                .setTangent(-Math.PI/2)
                .lineToY(pos_multiplier*(22+3.75))
                //Get First Sample
                .lineToY(pos_multiplier*(27+3.75))
                .splineToLinearHeading(new Pose2d(pos_multiplier*34,pos_multiplier*28.375,Math.PI), Math.PI/2)
                .setTangent(Math.PI)
                .lineToX(pos_multiplier*(36))
                //Go to Basket
                .splineToLinearHeading(new Pose2d(pos_multiplier*55.0294,pos_multiplier*55.0294,5*Math.PI/4), Math.PI)
                .splineToLinearHeading(new Pose2d(pos_multiplier*45,pos_multiplier*28.375,Math.PI), Math.PI/2)
                .splineToLinearHeading(new Pose2d(pos_multiplier*55.0294,pos_multiplier*55.0294,5*Math.PI/4), Math.PI)
                .splineToLinearHeading(new Pose2d(pos_multiplier*48,pos_multiplier*48,Math.PI), Math.PI)
                .splineToLinearHeading(new Pose2d(pos_multiplier*53,pos_multiplier*28.375,Math.PI), Math.PI/2)
                .splineToLinearHeading(new Pose2d(pos_multiplier*55.0294,pos_multiplier*55.0294,5*Math.PI/4), Math.PI)
                .splineToLinearHeading(new Pose2d(pos_multiplier*19,pos_multiplier*15,0), -Math.PI/2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}