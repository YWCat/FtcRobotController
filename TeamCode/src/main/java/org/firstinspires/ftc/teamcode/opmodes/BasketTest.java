package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name="BasketTest", group="Autonomous")
public class BasketTest extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;
    static double beginX = pos_multiplier*(24+botWidthHalf), beginY = pos_multiplier*(-botLengthHalf+72), beginH = Math.PI/2;
    static double basket_X = pos_multiplier*58, basket_Y = pos_multiplier*54, basket_H = Math.PI/4;
    static double sample1_X = pos_multiplier*53, sample_Y = pos_multiplier*(31+botLengthHalf), sample1_H = Math.PI/2;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d basketPose = new Pose2d(basket_X, basket_Y, basket_H);
        Pose2d sample1Pose = new Pose2d(sample1_X, sample_Y, sample1_H);
        Pose2d sample2Pose = new Pose2d(sample1_X-10, sample_Y, sample1_H);
        Pose2d sample3Pose = new Pose2d((sample1_X-12)+(botLengthHalf*Math.cos(45)), -29-(botLengthHalf*Math.sin(45)), 3*Math.PI/4);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Action basketTraj = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(basketPose, 3*Math.PI/2)
                .splineToLinearHeading(sample1Pose, Math.PI)
                .splineToLinearHeading(basketPose, Math.PI/2)
                .splineToLinearHeading(sample2Pose, Math.PI)
                .splineToLinearHeading(basketPose, Math.PI/2)
                .splineToLinearHeading(sample3Pose, Math.PI)
                .splineToLinearHeading(basketPose, Math.PI)
                .splineToLinearHeading(new Pose2d(pos_multiplier*(36+botWidthHalf),pos_multiplier*(24),0), Math.PI/4)
                .splineToLinearHeading(new Pose2d(pos_multiplier*18,pos_multiplier*(0),0), 0)
                .build();
        waitForStart();
        Actions.runBlocking(basketTraj);        
    }
}
