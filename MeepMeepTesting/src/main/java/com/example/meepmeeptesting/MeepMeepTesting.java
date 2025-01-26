package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double botWidthHalf = 7.25;
        double botLengthHalf = 7.5;
        double beginX = (botWidthHalf), beginY = -1*(-botLengthHalf+72), beginH = 0; //-Math.PI/2;
        double chamberX = beginX - 6, chamberY = (-15-botWidthHalf);
        Pose2d beginPose = new Pose2d(beginX,beginY,beginH);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)
                        .setTangent(Math.PI/2)
                        .splineToConstantHeading( new Vector2d(chamberX,chamberY),3*Math.PI/2)
                        //.lineTo(new Vector2d(beginX,-1*33))
                        .splineToLinearHeading(new Pose2d(-34*-1, 24*-1, 0), Math.PI/2)
                        .splineToLinearHeading(new Pose2d(48,-5,0), -Math.PI/2)
                        .setTangent(-Math.PI/2)
                        .lineTo(new Vector2d(48,-60))
                        .lineTo(new Vector2d(48,-12))
                        .splineToConstantHeading(new Vector2d(59, -13),-Math.PI/2)
                        .lineTo(new Vector2d(59,-60))
                        .lineTo(new Vector2d(59,-12))
                        .splineToConstantHeading(new Vector2d(70, -13),-Math.PI/2)
                        .lineTo(new Vector2d(70,-60))
                        .lineTo(new Vector2d(70,-50))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}