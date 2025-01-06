package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name="SpecTest", group="Autonomous")
public class SpecTest extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;
    static double beginX = pos_multiplier*(0-botLengthHalf), beginY = pos_multiplier*(-botWidthHalf+72), beginH = Math.PI;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(beginX, 29*pos_multiplier, beginH);
        Pose2d observPose = new Pose2d(-67*pos_multiplier, 60*pos_multiplier, 0);
        Pose2d repickPose = new Pose2d(-48*pos_multiplier, 72*pos_multiplier, 0);
        Pose2d adjustedPose = new Pose2d(0,-24,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //Drive Action
        Action startToChamber = drive.actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .lineToY(pos_multiplier*29)
                .setTangent(Math.PI)
                .build();
        Action sweepTheSamples = drive.actionBuilder(chamberPose)
                .lineToY(pos_multiplier*33)
                .splineToLinearHeading(new Pose2d(-34*pos_multiplier, 24*pos_multiplier, 0), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-49*pos_multiplier,8*pos_multiplier,0), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .lineToY(12*pos_multiplier)
                .splineToConstantHeading(new Vector2d(-59*pos_multiplier, -13),-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .lineToY(12*pos_multiplier)
                .splineToConstantHeading(new Vector2d(-67*pos_multiplier, -13),-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .build();
        Action repickSpec = drive.actionBuilder(observPose)
                .lineToX(48)
                .setTangent(-Math.PI/2)
                .lineToY(-72)
                .build();
        Action hangSndSample = drive.actionBuilder(observPose)
                .lineToY(-65)
                .splineToLinearHeading(new Pose2d(0,20*pos_multiplier,Math.PI),Math.PI/2)
                .build();
        Action goToPark = drive.actionBuilder(adjustedPose)
                .setTangent(-Math.PI/2)
                .lineToY(-48)
                .splineToConstantHeading(new Vector2d(50,65*pos_multiplier),3*Math.PI/4)
                .build();

        waitForStart();

    }
}
