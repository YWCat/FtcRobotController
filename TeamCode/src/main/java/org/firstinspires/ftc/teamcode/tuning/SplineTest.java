package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

public final class SplineTest extends LinearOpMode {
    Telemetry telemetry;
    private DcMotorEx armMotor = null;
    private Servo claw = null;
    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, RobotConfig.sampleServo);
        RotatingSlide rotatingSlide = new RotatingSlide();
        //telemetry.addData("Motor Value", armMotor.getCurrentPosition());

        Pose2d beginPose = new Pose2d(23.5/3, -62.5, Math.PI/2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Action driveToChamber = drive.actionBuilder(beginPose)
                    .lineToY(-23.5-8)
                    .build();
            Action preChamber = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_CHAMBER_PREP_TICKS, true);
            Action lowerSlide = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_RETRACT, true);
            //Action placeChamber = arm.getPlaceChamber();
            waitForStart();
            Actions.runBlocking(
                    //new SequentialAction(
                            new ParallelAction(
                                    preChamber,
                                    driveToChamber
                            )//,placeChamber
                    //)
            ); //TODO: force timeout when trajectory ends ? maybe add a new base class for action?

            Pose2d chamberPose = new Pose2d(23.5/3, -23.5-8, Math.PI/2);
            Action driveToSpikeMarks = drive.actionBuilder(chamberPose)
                    .lineToY(-23.5*5/3)
                    .splineTo(new Vector2d(23.5*2/3, -47),0)
                    .lineToX(23.5*4/3)
                    .splineToSplineHeading(new Pose2d(new Vector2d(23.5*5/3, -23.5*5/3), Math.PI), Math.PI/2)
                    .setTangent(Math.PI/2)
                    .lineToY(-23.5*2/3)
                    .splineToSplineHeading(new Pose2d(new Vector2d(47, -23.5*2/3), Math.PI), -Math.PI/2)
                    .setTangent(-Math.PI/2)
                    .lineToYConstantHeading(-62.5)
                    .build();
            Actions.runBlocking(
                    new ParallelAction(
                            lowerSlide,
                            driveToSpikeMarks
                    )
            );


        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }

    }
}
