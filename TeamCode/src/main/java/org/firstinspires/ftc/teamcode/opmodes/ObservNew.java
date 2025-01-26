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
import org.firstinspires.ftc.teamcode.subsystems.SampleIntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenIntake;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

@Config
@Autonomous(name="ObservNew", group="Autonomous")
public final class ObservNew extends LinearOpMode {
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;

    double beginX = (botLengthHalf), beginY = (-72+botWidthHalf), beginH = 0;
    double chamberX = beginX - 10, chamberY = (-18-botWidthHalf);

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(chamberX, chamberY, beginH);
        Pose2d observPose = new Pose2d(-67, 60, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        RobotCore robotCore  = new RobotCore(this);
        RotatingSlide rotatingSlide = new RotatingSlide();
        rotatingSlide.slide.setTolerance(1.0);
        SpecimenIntake specimenIntake = new SpecimenIntake(); //actually the most useless class, but its for the sake of abstraction
        SampleIntakeClaw sampleIntake = new SampleIntakeClaw();
        // Specimen Actions
        Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        Action openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);
        Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN, 1.0,  true);
        Action depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 1.0, true);
        //Action dropSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_PICK_UP_SPECIMEN_IN, 0.4, true);
        Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, true);
        Action goFwdABit = drive.actionBuilder(chamberPose)
                .setTangent(Math.PI/2)
                .lineToY(chamberY+(3))
                .build();
        //Drive Action
        Action startToChamber = drive.actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .splineToConstantHeading( new Vector2d(chamberX,chamberY),Math.PI/2)
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

        Action sweepTheSamples = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-1*33)
                .splineToLinearHeading(new Pose2d(36, 24*-1, 0), Math.PI/2)
                .splineToLinearHeading(new Pose2d(48,-4,0), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .lineToY(-60)
                .lineToY(-50)
                //.splineToConstantHeading(new Vector2d(54, -5),-Math.PI/2)
                //.lineToY(-60)
                //.lineToY(-10)
                //.splineToConstantHeading(new Vector2d(60, -5),-Math.PI/2)
                //.lineToY(-60)
                //.lineToY(-50)

                .build();
        //push the samples in the observation zone
        Actions.runBlocking(new ParallelAction(retractSlide, sweepTheSamples)
        );

        Action repickSpec = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .lineToX(48)
                .setTangent(Math.PI/2)
                .lineToY(-73.5)
                .build();
        Action specWristToPick= specimenIntake.getMoveSpecimenWrist(specimenIntake.INTAKE_WRIST, true);
        Actions.runBlocking(
                new ParallelAction(
                        repickSpec,
                        specWristToPick
                )
        );
        //Hang 2nd specimen on high chamber
        Action hangSndSample = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-65)
                .splineToLinearHeading(new Pose2d(2,chamberY,0),Math.PI/2)
                .build();
        Action specWristToPlace = specimenIntake.getMoveSpecimenWrist(specimenIntake.OUTTAKE_WRIST, true);
        raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN, 1.0,  true);
        closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 1.0, true);
        openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);
        // Close spec claw, move to chamber (raiseSilde, specWrist prep)
        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(closeSpecimen, new SleepAction(0.0)),
                        new ParallelAction(
                                hangSndSample,
                                raiseSlide,
                                specWristToPlace
                        )
                )
        );
        Action goBckABit = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(drive.pose.position.y-1)
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        goBckABit,
                        depositSlide,
                        new SequentialAction(new SleepAction(0.05),openSpecimen)
                )
        );

        // Go back to Observ zone to repick
        Action goToRepick2 = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-30)
                .splineToConstantHeading(new Vector2d(50,-73),-Math.PI/2) //make sure to hit wall
                .build();
        specWristToPick = specimenIntake.getMoveSpecimenWrist(specimenIntake.INTAKE_WRIST, true);
        retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, true);

        Actions.runBlocking(new ParallelAction(goToRepick2,specWristToPick,
                new SequentialAction(new SleepAction(0.3),retractSlide)));

        // Hang 3rd
        Pose2d SndRepickPose = new Pose2d(drive.pose.position.x,-72,0);
        Action hangThdSample = drive.actionBuilder(SndRepickPose)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(4,chamberY-3,0),Math.PI/2)
                .build();
        raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN, 1.0,  true);
        closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        specWristToPlace = specimenIntake.getMoveSpecimenWrist(specimenIntake.OUTTAKE_WRIST, true);
        depositSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 1.0, true);
        openSpecimen= specimenIntake.getMoveSpecimenIntake(specimenIntake.OPEN, true);


        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(new SleepAction(0.5),closeSpecimen),
                        new ParallelAction(
                                hangThdSample,
                                raiseSlide,
                                specWristToPlace
                        )
                )
        );
        goBckABit = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(drive.pose.position.y-1)
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        goBckABit,
                        depositSlide,
                        new SequentialAction(new SleepAction(0.05),openSpecimen)
                )
        );
        specWristToPick = specimenIntake.getMoveSpecimenWrist(specimenIntake.INTAKE_WRIST, true);



        Action goToPark = drive.actionBuilder(drive.pose)
                .setTangent(Math.PI/2)
                .lineToY(-33)
                .splineToLinearHeading(new Pose2d(45,-60, 0),Math.PI/2)
                .build();
        retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, true);
        closeSpecimen = specimenIntake.getMoveSpecimenIntake(specimenIntake.CLOSE, true);
        Actions.runBlocking(new ParallelAction(goToPark, specWristToPick, new SequentialAction(new SleepAction(0.3),retractSlide),closeSpecimen));

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    }
}
