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
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SampleIntake;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenIntake;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

@Config
@Autonomous(name="RedBasket", group="Autonomous")
public final class RedBasket extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;

    static double beginX = pos_multiplier*(botWidthHalf), beginY = pos_multiplier*(-botLengthHalf+72), beginH = -Math.PI*pos_multiplier;
    static double chamberX = beginX, chamberY = pos_multiplier*(18+botWidthHalf), chamberH = beginH;
    static double firstSample_X = pos_multiplier*39.5, Sample_Y = pos_multiplier*18.375, Sample_H = -Math.PI*pos_multiplier; //X:38
    static double basket_X = pos_multiplier*56, basket_Y = pos_multiplier*53, basket_H = Math.PI+(-Math.PI/4*pos_multiplier);
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(beginX, chamberY, beginH);
        Pose2d fstSamplePose = new Pose2d(firstSample_X, Sample_Y, Sample_H);
        Pose2d sndSamplePose = new Pose2d(firstSample_X+(pos_multiplier*11.5), Sample_Y-pos_multiplier*3, Sample_H);
        Pose2d thdSamplePose = new Pose2d(firstSample_X+(pos_multiplier*20), Sample_Y, Sample_H);
        Pose2d basketPose = new Pose2d(basket_X, basket_Y, basket_H);
        Pose2d basketBackPose = new Pose2d(pos_multiplier*48,pos_multiplier*48,Math.PI*pos_multiplier);
        Pose2d basketBack2Pose = new Pose2d(pos_multiplier*36, pos_multiplier*36, pos_multiplier* Math.PI);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        RobotCore robotCore  = new RobotCore(this);
        RotatingSlide rotatingSlide = new RotatingSlide();
        SpecimenIntake specimenIntake = new SpecimenIntake(); //actually the most useless class, but its for the sake of abstraction
        SampleIntake sampleIntake = new SampleIntake();

        // Specimen Actions
        Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        Action openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);
        Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN, 1,  true);
        Action depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 0.2, true);
        Action dropSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_PICK_UP_SPECIMEN_IN, 0.4, true);

        // intake actions

        Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS-100, true);
        Action wristIntake = sampleIntake.getTurnWristAction(sampleIntake.WRIST_INTAKE_ROLLER, true);
        Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, true);
        Action rollerOn = sampleIntake.getStartRollerAction(true, true);

        // Drive actions
        Action beginToChamber = drive.actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY)
                .build();
        Action ChamberToFstSample = drive.actionBuilder(chamberPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY+(pos_multiplier*7))
                .splineToLinearHeading(fstSamplePose, Math.PI/2)
                .build();
        Action FstSampleToBasket = drive.actionBuilder(fstSamplePose)
                .splineToLinearHeading(basketPose, Math.PI)
                .build();
        Action BasketToSndSample = drive.actionBuilder(basketPose)
                .splineToLinearHeading(basketBackPose, Math.PI)
                .splineToLinearHeading(basketBack2Pose, Math.PI/2)
                .splineToLinearHeading(sndSamplePose, Math.PI/2)
                .build();
        Action SndSampleToBasket = drive.actionBuilder(sndSamplePose)
                .splineToLinearHeading(basketPose, Math.PI)
                .build();
        Action BasketToThdSample = drive.actionBuilder(basketPose)
                .splineToLinearHeading(new Pose2d(pos_multiplier*48,pos_multiplier*48,Math.PI), Math.PI)
                .splineToLinearHeading(basketBack2Pose, Math.PI/2)
                .splineToLinearHeading(sndSamplePose, Math.PI/2)
                .build();
        Action ThdSampleToBasket = drive.actionBuilder(thdSamplePose)
                .splineToLinearHeading(new Pose2d(basket_X,basket_Y,basket_H), Math.PI)
                .build();
        Action goToAscent = drive.actionBuilder(basketPose)
                .splineToLinearHeading(new Pose2d(pos_multiplier*19,pos_multiplier*10,0), -Math.PI/2)
                .build();

        waitForStart();

        // strafe to chamber and deposit specimen
        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            beginToChamber,
                            new SequentialAction(closeSpecimen, raiseSlide)
                    ),
                        depositSlide,
                        openSpecimen
                )
        );
        // spline to 1st sample, prep for intake
        Actions.runBlocking(
                new ParallelAction(
                        ChamberToFstSample,
                        retractSlide,
                        armToIntake,
                        wristIntake
                )
        );
        // Intake
        Action armToIntakeDip = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS+300, true);
        Actions.runBlocking(
                new ParallelAction(
                        armToIntakeDip,
                        rollerOn,
                        new SleepAction(0.8)
                )
        );

        // 1st sample to basket, extend slides, and rotate arm
        Action armToOut = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AUTO_BASKET_TICKS, true);
        Action outtakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN+2, 1,  true);
        Action rollerOut = sampleIntake.getReverseRollerAction(1500,true);//timeout in ms
        Action wristUp = sampleIntake.getTurnWristAction(0.4, true);
        retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            FstSampleToBasket,
                            armToOut
                        ),
                        outtakeSlide,
                        rollerOut,
                        wristUp,
                        retractSlide
                )
        );

        // Basket to 2nd sample
        armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS-100, true);
        wristIntake = sampleIntake.getTurnWristAction(sampleIntake.WRIST_INTAKE_ROLLER, true);
        Actions.runBlocking(
                new ParallelAction(
                        BasketToSndSample,
                        armToIntake,
                        wristIntake
                )
        );
        // 2nd sample Intake
        armToIntakeDip = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS+300, true);
        rollerOn = sampleIntake.getStartRollerAction(true, true);
        Actions.runBlocking(
                new ParallelAction(
                        armToIntakeDip,
                        rollerOn,
                        new SleepAction(0.8)
                )
        );

        // 2nd sample to basket, extend slides, and rotate arm
        armToOut = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AUTO_BASKET_TICKS, true);
        outtakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN+2, 1,  true);
        rollerOut = sampleIntake.getReverseRollerAction(1500,true);//timeout in ms
        wristUp = sampleIntake.getTurnWristAction(0.4, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                SndSampleToBasket,
                                armToOut
                        ),
                        outtakeSlide,
                        rollerOut,
                        wristUp,
                        retractSlide
                )
        );
        /*

        // Basket to 3rd sample
        armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS-100, true);
        wristIntake = sampleIntake.getTurnWristAction(sampleIntake.WRIST_INTAKE_ROLLER, true);
        Actions.runBlocking(
                new ParallelAction(
                        BasketToThdSample,
                        armToIntake,
                        wristIntake
                )
        );
        // 3rd sample Intake
        armToIntakeDip = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS+300, true);
        rollerOn = sampleIntake.getStartRollerAction(true, true);
        Actions.runBlocking(
                new ParallelAction(
                        armToIntakeDip,
                        rollerOn,
                        new SleepAction(0.8)
                )
        );
        // 3rd sample to basket, extend slides, and rotate arm
        armToOut = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AUTO_BASKET_TICKS, true);
        outtakeSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN+2, 1,  true);
        rollerOut = sampleIntake.getReverseRollerAction(500,true);//timeout in ms
        wristUp = sampleIntake.getTurnWristAction(0.4, true);
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                ThdSampleToBasket,
                                armToOut
                        ),
                        outtakeSlide,
                        new ParallelAction(
                                rollerOut,
                                new SleepAction(1)
                        ),
                        wristUp,
                        retractSlide
                )
        );

        // Basket to low rung ascend
        Actions.runBlocking(
                new SequentialAction(
                        goToAscent
                )
        );

         */

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    }
}
