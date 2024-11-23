package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@TeleOp(name="RedBasketTrajectory", group="Linear OpMode")
public final class RedBasketTrajectory extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(6.085+0, -3.75+72, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //Deposit Preload
                        .setTangent(-Math.PI/2)
                        .lineToY(22+3.75)
                        //Get First Sample
                        .lineToY(27+3.75)
                        .splineToSplineHeading(new Pose2d(32,28.375,0), 3*Math.PI/2)
                        .splineToConstantHeading(new Vector2d(49-6.085,26+3.75), Math.PI/2)
                        //Go to Basket
                        .splineToLinearHeading(new Pose2d(52,48,Math.PI/4), Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55.0294,55.0294), Math.PI/2)
                        .build()
        );
        //Go back to the submersible and hang
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(55.0294,55.0294,Math.PI/4))
                        .splineToLinearHeading(new Pose2d(6.085,26+3.75,-Math.PI/2), -Math.PI/2)
                        .build()
        );

        /*
        //Pick up the second sample
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToSplineHeading(new Pose2d(45,39.5147,-Math.PI/2),Math.PI/2)
                        .splineToSplineHeading(new Pose2d(51.915,27.75,Math.PI), 0)
        );

         */
        /*
        //Go to the basket
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(48,48,(3*Math.PI)/4), 0)
                        .splineToConstantHeading(new Vector2d(55.0294,55.0294),(3*Math.PI)/4)
        );
        //Pick up the third sample
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToSplineHeading(new Pose2d(50,39.5147,-Math.PI/2),Math.PI/2)
                        .splineToSplineHeading(new Pose2d(61.915,27.75,Math.PI), 0)
        );
        //Go to the basket
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(48,48,(3*Math.PI)/4), 0)
                        .splineToConstantHeading(new Vector2d(55.0294,55.0294),(3*Math.PI)/4)
        );

         */


    }
}
