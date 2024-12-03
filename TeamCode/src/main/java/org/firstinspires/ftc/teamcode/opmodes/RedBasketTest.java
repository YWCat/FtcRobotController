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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SampleIntake;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenIntake;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

@Config
@Autonomous(name="RedBasketTest", group="Autonomous")
public final class RedBasketTest extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;

    static double beginX = pos_multiplier*(botWidthHalf), beginY = pos_multiplier*(-botLengthHalf+72), beginH = -Math.PI*pos_multiplier;
    static double chamberX = beginX, chamberY = pos_multiplier*(18+botWidthHalf), chamberH = beginH;
    static double firstSample_X = 35.5, Sample_Y = 18.375, Sample_H = -Math.PI*pos_multiplier; //X:38
    static double basket_X = 58.8674, basket_Y = 56.1745, basket_H = Math.PI+(-Math.PI/4*pos_multiplier);
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(beginX, chamberY, beginH);
        Pose2d fstSamplePose = new Pose2d(firstSample_X, Sample_Y, Sample_H);
        Pose2d sndSamplePose = new Pose2d(firstSample_X+(pos_multiplier*10), Sample_Y, Sample_H);
        Pose2d thdSamplePose = new Pose2d(firstSample_X+(pos_multiplier*20), Sample_Y, Sample_H);
        Pose2d basketPose = new Pose2d(basket_X, basket_Y, basket_H);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        RobotCore robotCore  = new RobotCore(this);
        RotatingSlide rotatingSlide = new RotatingSlide();
        SpecimenIntake specimenIntake = new SpecimenIntake(); //actually the most useless class, but its for the sake of abstraction
        SampleIntake sampleIntake = new SampleIntake();

        // Specimen Actions
        Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        Action openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);
        Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN+2, 0.4,  true);
        Action depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 0.2, true);
        Action dropSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_PICK_UP_SPECIMEN_IN, 0.4, true);

        // intake actions

        Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS, true);
        Action armToIntakeDip = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS+50, true);
        Action wristIntake = sampleIntake.getTurnWristAction(sampleIntake.WRIST_INTAKE_ROLLER+0.1, true);
        Action wristOuttake = sampleIntake.getTurnWristAction(sampleIntake.WRIST_OUTTAKE_ROLLER, true);
        Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, true);
        Action prepIntake = sampleIntake.getPrepIOAction(true, true);
        Action rollerOn = sampleIntake.getStartRollerAction(true, true);
        Action toIntake = new ParallelAction( rollerOn, new SequentialAction(armToIntake, wristIntake));

        // Drive actions
        Action beginToChamber = drive.actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY)
                .build();

        Action ChamberToFstSample = drive.actionBuilder(chamberPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY+(pos_multiplier*7))
                .splineToLinearHeading(new Pose2d(pos_multiplier*firstSample_X,pos_multiplier* Sample_Y,Math.PI), Math.PI/2)
                .build();

        Action FstSampleToBasket = drive.actionBuilder(fstSamplePose)
                .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,basket_H), Math.PI)
                .build();
        Action FstSampleSleep = drive.actionBuilder(fstSamplePose)
                .waitSeconds(3)
                .build();

        Action BasketToSndSample = drive.actionBuilder(basketPose)
                .splineToLinearHeading(new Pose2d(pos_multiplier*(firstSample_X+10),pos_multiplier* Sample_Y,Math.PI), Math.PI/2)
                .build();

        Action SndSampleToBasket = drive.actionBuilder(sndSamplePose)
                .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,basket_H), Math.PI)
                .build();

        Action BasketToThdSample = drive.actionBuilder(basketPose)
                .splineToLinearHeading(new Pose2d(pos_multiplier*48,pos_multiplier*48,Math.PI), Math.PI)
                .splineToLinearHeading(new Pose2d(pos_multiplier*(firstSample_X+20),pos_multiplier* Sample_Y,Math.PI), Math.PI/2)
                .build();

        Action ThdSampleToBasket = drive.actionBuilder(thdSamplePose)
                .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,basket_H), Math.PI)
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
                            new SequentialAction(closeSpecimen) //, raiseSlide)
                    ),
                        //depositSlide,
                        openSpecimen
                )
        );

        // spline to 1st sample, then intake
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                ChamberToFstSample,
                                new SequentialAction(
                                       dropSlide,
                                       new ParallelAction(
                                               armToIntake,
                                               wristIntake,
                                               rollerOn
                                       )
                                )
                        ),
                        new ParallelAction(
                                armToIntakeDip,
                                FstSampleSleep
                        )
                )
        );


/*
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //Deposit Preload
                        .setTangent(Math.PI/2)
                        .lineToY(pos_multiplier*(19+botLengthHalf))

                        //Get First Sample
                        .lineToY(pos_multiplier*(27+3.75))
                        .splineToLinearHeading(new Pose2d(pos_multiplier*firstSample_X,pos_multiplier* Sample_Y,Math.PI), Math.PI/2)
                        //Go to Basket
                        .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,5*Math.PI/4), Math.PI)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*(firstSample_X+10),pos_multiplier* Sample_Y,Math.PI), Math.PI/2)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,5*Math.PI/4), Math.PI)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*48,pos_multiplier*48,Math.PI), Math.PI)

                        .splineToLinearHeading(new Pose2d(pos_multiplier*(firstSample_X+20),pos_multiplier* Sample_Y,Math.PI), Math.PI/2)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,5*Math.PI/4), Math.PI)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*19,pos_multiplier*10,0), -Math.PI/2)


                        .build());

 */
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    }
}
