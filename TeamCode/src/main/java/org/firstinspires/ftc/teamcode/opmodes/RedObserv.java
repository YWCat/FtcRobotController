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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SampleIntake;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenIntake;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

@Config
@Autonomous(name="RedObserv", group="Autonomous")
public final class RedObserv extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;

    static double beginX = pos_multiplier*(-24), beginY = pos_multiplier*(-botLengthHalf+72), beginH = -Math.PI*pos_multiplier;
    static double chamberX = pos_multiplier*(-2-botLengthHalf), chamberY = pos_multiplier*(15.5+botWidthHalf), chamberH = beginH;
    static double firstSample_X = -55*pos_multiplier, Sample_Y = 13*pos_multiplier, Sample_H = 0; //X:38

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(beginX, beginY, beginH);
        Pose2d chamberPose = new Pose2d(chamberX, chamberY,chamberH);
        Pose2d fstSamplePose = new Pose2d(firstSample_X, Sample_Y, Sample_H);
        Pose2d sndSamplePose = new Pose2d(firstSample_X+(pos_multiplier*12), Sample_Y, Sample_H);
        Pose2d thdSamplePose = new Pose2d(firstSample_X+(pos_multiplier*9.6), Sample_Y, Sample_H);

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
        //Action dropSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_PICK_UP_SPECIMEN_IN, 0.4, true);
        Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, true);
        // Drive actions
        Action beginToChamber =  drive.actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(chamberX,chamberY),Math.PI/2)
                .build();

        Action chamberToObserv = drive.actionBuilder(chamberPose)
                .setTangent(Math.PI/2)
                .lineToY(pos_multiplier*(27+3.75))
                .splineToLinearHeading(new Pose2d(-35*pos_multiplier, 36*pos_multiplier, 0), Math.PI/2)
                .splineToSplineHeading(new Pose2d(-55*pos_multiplier,Sample_Y,0), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .setTangent(Math.PI/2)
                .lineToY(12*pos_multiplier)
                .splineToLinearHeading(new Pose2d(-67*pos_multiplier, Sample_Y,0),-Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(60*pos_multiplier)
                .setTangent(Math.PI/2)
                .lineToY(12*pos_multiplier)
                .splineToLinearHeading(new Pose2d(-76.5*pos_multiplier, Sample_Y,0),-Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(67*pos_multiplier)
                .build();

        waitForStart();
        // Move to chamber and deposit specimen
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(2),
                        new ParallelAction(
                                beginToChamber,
                                new SequentialAction(closeSpecimen, raiseSlide)
                        ),
                        depositSlide,
                        openSpecimen
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        retractSlide,
                        chamberToObserv
                )
        );



        // Complete trajectory
        /*
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(chamberX,chamberY),Math.PI/2)
                .lineToY(pos_multiplier*(27+3.75))
                .splineToLinearHeading(new Pose2d(-35*pos_multiplier, 36*pos_multiplier, 0), Math.PI/2)
                .splineToSplineHeading(new Pose2d(-55*pos_multiplier,Sample_Y,0), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .lineToY(60*pos_multiplier)
                .setTangent(Math.PI/2)
                .lineToY(12*pos_multiplier)
                .splineToLinearHeading(new Pose2d(-67*pos_multiplier, Sample_Y,0),-Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(60*pos_multiplier)
                .setTangent(Math.PI/2)
                .lineToY(12*pos_multiplier)
                .splineToLinearHeading(new Pose2d(-76.5*pos_multiplier, Sample_Y,0),-Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(67*pos_multiplier)
                .build());
*/

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    }
}
