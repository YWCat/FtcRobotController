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
import org.firstinspires.ftc.teamcode.subsystems.SampleIntakeRoller;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenIntake;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

@Config
@Autonomous(name="RedObserv", group="Autonomous")
public final class RedObserv extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;

    static double beginX = pos_multiplier*(0-botLengthHalf), beginY = pos_multiplier*(-botWidthHalf+72), beginH = Math.PI;
    static double chamberY = pos_multiplier*(19+botWidthHalf);

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(beginX, chamberY, beginH);
        Pose2d observPose = new Pose2d(-67*pos_multiplier, 60*pos_multiplier, 0);
        Pose2d adjustedPose = new Pose2d(0,-24,0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        RobotCore robotCore  = new RobotCore(this);
        RotatingSlide rotatingSlide = new RotatingSlide();
        SpecimenIntake specimenIntake = new SpecimenIntake(); //actually the most useless class, but its for the sake of abstraction

        // Specimen Actions
        Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        Action openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);
        Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN+2, 1.0,  true);
        Action depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN+2, 0.3, true);
        //Action dropSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_PICK_UP_SPECIMEN_IN, 0.4, true);
        Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN+2, 1, true);
        Action goFwdABit = drive.actionBuilder(chamberPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY-(1*pos_multiplier))
                .build();
        //Drive Action
        Action startToChamber = drive.actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY)
                .build();
        Action sweepTheSamples = drive.actionBuilder(chamberPose)
                .setTangent(Math.PI/2)
                .lineToY(pos_multiplier*33)
                .splineToLinearHeading(new Pose2d(-34*pos_multiplier, 24*pos_multiplier, 0), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-48*pos_multiplier,5*pos_multiplier,0), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .lineToY(12*pos_multiplier)
                .splineToConstantHeading(new Vector2d(-59*pos_multiplier, -13),-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .lineToY(58*pos_multiplier)
                .build();

        waitForStart();
        // Move to chamber and deposit specimen
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                startToChamber,
                                closeSpecimen,
                                raiseSlide
                        ),
                        goFwdABit,
                        depositSlide,
                        new SequentialAction(openSpecimen,new SleepAction(0.05))
                )
        );
        //push the samples in the observation zone
        Actions.runBlocking(new ParallelAction(retractSlide, sweepTheSamples)
        );

        Action repickSpec = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .lineToX(48)
                .setTangent(Math.PI/2)
                .lineToY(-71)
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        repickSpec,
                        openSpecimen
                )
        );

        //Hang specimen on high chamber
        Action hangSndSample = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-65)
                .splineToLinearHeading(new Pose2d(0,chamberY-3,Math.PI),Math.PI/2)
                .build();

        raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN+2, 1.0,  true);
        closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 0.3, true);
        openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);

        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(closeSpecimen, new SleepAction(0.3)),
                        new ParallelAction(
                                hangSndSample,
                                raiseSlide
                        ),
                        depositSlide,
                        openSpecimen
                )
        );

        Action goToPark = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-30)
                .splineToLinearHeading(new Pose2d(45,69*pos_multiplier, 0),-Math.PI/2)
                .build();
        retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN+1.75, 1, true);
        Actions.runBlocking(new ParallelAction(goToPark,retractSlide));
        /*
        Action hangThdSample = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-65)
                .splineToLinearHeading(new Pose2d(-4,chamberY-3,Math.PI),Math.PI/2)
                .build();
        raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN+2, 1.0,  true);
        closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 0.3, true);
        openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);

        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(closeSpecimen, new SleepAction(0.3)),
                        new ParallelAction(
                                hangSndSample,
                                raiseSlide
                        ),
                        depositSlide,
                        openSpecimen
                )
        );
        */

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    }
}
