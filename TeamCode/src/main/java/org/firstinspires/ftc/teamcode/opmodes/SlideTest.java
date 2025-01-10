package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightColor;
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SampleIntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenIntake;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

@Config
@Autonomous(name="SlideTest", group="Autonomous")
public class SlideTest extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;
    static double beginX = pos_multiplier*(24+botWidthHalf), beginY = pos_multiplier*(-botLengthHalf+72), beginH = Math.PI/2;
    static double basket_X = pos_multiplier*(59), basket_Y = pos_multiplier*(55), basket_H = Math.PI/4;
    static double sample1_X = pos_multiplier*(52-1), sample_Y = pos_multiplier*(32+7+botLengthHalf), sample1_H = Math.PI/2;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d basketPose = new Pose2d(basket_X, basket_Y, basket_H);
        Pose2d sample1Pose = new Pose2d(sample1_X, sample_Y, sample1_H);
        Pose2d sample2Pose = new Pose2d(sample1_X+pos_multiplier*(12), sample_Y+pos_multiplier*(1), sample1_H);
        Pose2d sample3Pose = new Pose2d(sample1_X+pos_multiplier*(12), sample_Y+pos_multiplier*(-8), Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        RobotCore robotCore  = new RobotCore(this);
        LimeLightColor LLCamClr = new LimeLightColor(hardwareMap);
        //LLCam.xOffset = pos_multiplier*botLengthHalf;
        drive.LLCamClr = LLCamClr;
        RotatingSlide rotatingSlide = new RotatingSlide();
        //rotatingSlide.slide.setTolerance(1.0);
        SpecimenIntake specimenIntake = new SpecimenIntake(); //actually the most useless class, but its for the sake of abstraction
        SampleIntakeClaw sampleIntake = new SampleIntakeClaw();

        waitForStart();

        // To Basket, and dump preload sample
        Action toBasket = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(basketPose, 3*Math.PI/2)
                .build();
        Action armToOut = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AUTO_BASKET_TICKS_CLAW, true);

        Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN+2+10, 1.0,  true);
        Action outtakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN-2, 1.0,  true);

        Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1,  true);
        Action wristOutPrep = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_PREP_OUTTAKE_CLAW, true);
        Action wristOut = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_OUTTAKE_CLAW, true);
        Action openClaw = sampleIntake.getMoveClawAction(false, true);
        Action specimenOpen = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.OPEN, true);

        Actions.runBlocking(new SequentialAction(
                //outtakeSlide,
                raiseSlide,
                new SleepAction(1.0),
                retractSlide,
                new SleepAction(1.0),
                outtakeSlide,
                new SleepAction(1.0)
        ));

        retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1,  true);
        Actions.runBlocking(retractSlide);

    }
}
