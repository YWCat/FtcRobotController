package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
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
@Autonomous(name="BasketTestLL", group="Autonomous")
public class BasketTestLL extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;
    static double beginX = pos_multiplier*(24+botWidthHalf), beginY = pos_multiplier*(-botLengthHalf+72), beginH = Math.PI/2;
    static double basket_X = pos_multiplier*(59+1), basket_Y = pos_multiplier*(55+1), basket_H = Math.PI/4;
    static double sample1_X = pos_multiplier*(52-1), sample_Y = pos_multiplier*(32+7+botLengthHalf), sample1_H = Math.PI/2;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d basketPose = new Pose2d(basket_X, basket_Y, basket_H);
        Pose2d basket1Pose = new Pose2d(basket_X+2, basket_Y+2, basket_H);
        Pose2d basket2Pose = new Pose2d(basket_X+1, basket_Y+1, basket_H);
        Pose2d basket3Pose = new Pose2d(basket_X+2, basket_Y+2, basket_H);
        Pose2d sample1Pose = new Pose2d(sample1_X, sample_Y, sample1_H);
        Pose2d sample2Pose = new Pose2d(sample1_X+pos_multiplier*(13), sample_Y+pos_multiplier*(0), sample1_H);
        Pose2d sample3Pose = new Pose2d(sample1_X+pos_multiplier*(12), sample_Y+pos_multiplier*(-8), Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.endWErr = true;

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
        Action outtakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN-3, 1.0,  true);
        Action wristOutPrep = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_PREP_OUTTAKE_CLAW, true);
        Action wristOut = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_OUTTAKE_CLAW, true);
        Action openClaw = sampleIntake.getMoveClawAction(false, true);
        Action specimenOpen = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.OPEN, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                toBasket,
                                wristOutPrep,
                                new SequentialAction(armToOut, outtakeSlide),
                                specimenOpen
                        ),
                        new ParallelAction(wristOut, new SleepAction(0.1)),
                        new ParallelAction(openClaw, new SleepAction(0.3))
                )
        );

        // basket To 1st sample, and pick up sample
        Action bskToFst = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(sample1Pose, Math.PI)
                .build();
        Action armToIntakePrep = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_DEG-10, true);
        Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_DEG-3, true);
        Action intakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN+7, 0.8, true);
        Action wristIntake = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_INTAKE_CLAW, true);
        Action closeClaw = sampleIntake.getMoveClawAction(true, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                bskToFst,
                                intakeSlide,
                                armToIntakePrep,
                                wristIntake
                        )
                )
        );
        drive.setCamCorr(true, 0.9, 1,0, -7, 2);
        drive.alignByCam(true);
        drive.alignByCam(false);
        drive.disCamCorr();

        Actions.runBlocking(new SequentialAction(
                armToIntake,
                new ParallelAction(closeClaw, new SleepAction(0.3))
        ));

        // To Basket, and dump 1st sample
        toBasket = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(basket1Pose, Math.PI/2)
                .build();
        armToOut = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AUTO_BASKET_TICKS_CLAW, true);
        outtakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN-3, 1.0,  true);
        wristOutPrep = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_PREP_OUTTAKE_CLAW, true);
        wristOut = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_OUTTAKE_CLAW, true);
        openClaw = sampleIntake.getMoveClawAction(false, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                toBasket,
                                wristOutPrep,
                                new SequentialAction(armToOut, outtakeSlide)
                        ),
                        new ParallelAction(wristOut, new SleepAction(0.1)),
                        new ParallelAction(openClaw, new SleepAction(0.3))
                )
        );

        // basket To 2nd sample, and pick up sample
        Action bskToSnd = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(sample2Pose, Math.PI)
                .build();
        armToIntakePrep = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_DEG-10, true);
        armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_DEG-2, true);
        intakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN+7, 0.8, true);
        wristIntake = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_INTAKE_CLAW, true);
        closeClaw = sampleIntake.getMoveClawAction(true, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                bskToSnd,
                                intakeSlide,
                                armToIntakePrep,
                                wristIntake
                        )
                )
        );
        drive.setCamCorr(true, 0.9, 1,0, -7, 2);
        drive.alignByCam(true);
        drive.alignByCam(false);
        drive.disCamCorr();
        Actions.runBlocking(new SequentialAction(
                armToIntake,
                new ParallelAction(closeClaw, new SleepAction(0.3))
        ));

        // To Basket, and dump 2nd sample
        toBasket = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(basket2Pose, Math.PI/2)
                .build();
        armToOut = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AUTO_BASKET_TICKS_CLAW, true);
        outtakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN-3, 1.0,  true);
        wristOutPrep = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_PREP_OUTTAKE_CLAW, true);
        wristOut = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_OUTTAKE_CLAW, true);
        openClaw = sampleIntake.getMoveClawAction(false, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                toBasket,
                                wristOutPrep,
                                new SequentialAction(armToOut, outtakeSlide)
                        ),
                        new ParallelAction(wristOut, new SleepAction(0.1)),
                        new ParallelAction(openClaw, new SleepAction(0.3))
                )
        );

        // basket To 3rd sample, and pick up sample
        Action bskToThrd = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(sample3Pose, Math.PI)
                .build();
        Action armToVertical = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_VERTICAL_POS, true);
        armToIntakePrep = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_DEG-12, true);
        armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_DEG-2, true);
        intakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN+4, 0.8, true);
        wristIntake = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_INTAKE_CLAW, true);
        closeClaw = sampleIntake.getMoveClawAction(true, true);
        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            bskToThrd,
                            intakeSlide,
                            armToVertical,
                            wristIntake
                            ),
                        armToIntakePrep )
        );
        drive.setCamCorr(true, 1, 1,-3, -9.5, 2);
        drive.alignByCam(true);
        drive.alignByCam(false);
        drive.disCamCorr();

        Actions.runBlocking(new SequentialAction(
                armToIntake,
                new ParallelAction(closeClaw, new SleepAction(0.3))
        ));

        // To Basket, and dump 3rd sample
        toBasket = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(basket3Pose, Math.PI/2)
                .build();
        armToOut = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AUTO_BASKET_TICKS_CLAW, true);
        outtakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN-2, 1.0,  true);
        wristOutPrep = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_PREP_OUTTAKE_CLAW, true);
        wristOut = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_OUTTAKE_CLAW, true);
        openClaw = sampleIntake.getMoveClawAction(false, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                toBasket,
                                wristOutPrep,
                                new SequentialAction(armToOut, outtakeSlide)
                        ),
                        new ParallelAction(wristOut, new SleepAction(0.1)),
                        new ParallelAction(openClaw, new SleepAction(0.3))
                )
        );

        // Basket to park
        Action toAscend = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(pos_multiplier*37,pos_multiplier*(15),Math.PI/2), Math.PI/2)
                .splineToLinearHeading(new Pose2d(pos_multiplier*18,pos_multiplier*(0),0), 0)
                .build();
        armToVertical = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AUTO_ASCEND_DEG, true);
        intakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN+10, 1.0, true);
        wristIntake = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_ASCEND, true);
        closeClaw = sampleIntake.getMoveClawAction(true, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                toAscend,
                                intakeSlide,
                                armToVertical,
                                wristIntake
                        )
                )
        );

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
