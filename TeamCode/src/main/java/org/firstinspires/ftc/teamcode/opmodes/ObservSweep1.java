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
import org.firstinspires.ftc.teamcode.subsystems.SpecimenIntake;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

@Config
@Autonomous(name="ObservSwp1", group="Autonomous")
public final class ObservSweep1 extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;

    static double beginX = pos_multiplier*(0-botLengthHalf), beginY = pos_multiplier*(-botWidthHalf+72), beginH = Math.PI;
    static double chamberY = pos_multiplier*(18+botWidthHalf);

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(beginX-1, chamberY, beginH);
        Pose2d observPose = new Pose2d(-67*pos_multiplier, 60*pos_multiplier, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        RobotCore robotCore  = new RobotCore(this);
        RotatingSlide rotatingSlide = new RotatingSlide();
        rotatingSlide.slide.setTolerance(1.0);
        SpecimenIntake specimenIntake = new SpecimenIntake(); //actually the most useless class, but its for the sake of abstraction

        // Specimen Actions
        Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        Action openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);
        Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN, 1.0,  true);
        Action depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 0.3, true);
        //Action dropSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_PICK_UP_SPECIMEN_IN, 0.4, true);
        Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN+2, 1, true);
        Action goFwdABit = drive.actionBuilder(chamberPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY-(1*pos_multiplier))
                .build();
        //Drive Action
        Action startToChamber = drive.actionBuilder(beginPose)
                //.setTangent(Math.PI/2)
                //.lineToY(chamberY)
                .splineToConstantHeading( new Vector2d(0+botLengthHalf-1,chamberY),3*Math.PI/2)
                .build();
        Action sweepTheSamples = drive.actionBuilder(chamberPose)
                .setTangent(Math.PI/2)
                .lineToY(pos_multiplier*33)
                .splineToLinearHeading(new Pose2d(-36*pos_multiplier, 24*pos_multiplier, 0), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-48*pos_multiplier,5*pos_multiplier,0), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .lineToY(45*pos_multiplier)
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
                .splineToLinearHeading(new Pose2d(0,chamberY-2,Math.PI),Math.PI/2)
                .build();

        raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN+2, 1.0,  true);
        closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 0.3, true);
        openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);

        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(closeSpecimen, new SleepAction(0.5)),
                        new ParallelAction(
                                hangSndSample,
                                raiseSlide
                        )
                )
        );
        Pose2d adjustedPose = new Pose2d(drive.pose.position.x,-24,Math.PI);
        Action backUpABit = drive.actionBuilder(adjustedPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY-5)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        backUpABit,
                        depositSlide,
                        openSpecimen
                )
        );

        Action goToRepick2 = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-30)
                .splineToLinearHeading(new Pose2d(50,69*pos_multiplier, 0),-Math.PI/2)
                .build();
        retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN+1.75, 1, true);

        Actions.runBlocking(new ParallelAction(goToRepick2,new SequentialAction(new SleepAction(0.3),retractSlide)));
        Action hangThdSample = drive.actionBuilder(drive.pose)
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
                        new SequentialAction(closeSpecimen, new SleepAction(0.5)),
                        new ParallelAction(
                                hangThdSample,
                                raiseSlide
                        )
                )
        );
        Pose2d adjustedPose2 = new Pose2d(drive.pose.position.x,-24,Math.PI);
        Action backUpABit2 = drive.actionBuilder(adjustedPose2)
                .setTangent(Math.PI/2)
                .lineToY(chamberY-4)
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        backUpABit2,
                        depositSlide,
                        openSpecimen
                )
        );

        Action goToPark = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-30)
                .splineToLinearHeading(new Pose2d(45,69*pos_multiplier, 0),-Math.PI/2)
                .build();
        retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, true);
        closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        Actions.runBlocking(new ParallelAction(goToPark,new SequentialAction(new SleepAction(0.3),retractSlide),closeSpecimen));

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    }
}
