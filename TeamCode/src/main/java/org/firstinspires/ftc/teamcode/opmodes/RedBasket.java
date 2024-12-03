package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@TeleOp(name="RedBasketTrajectory", group="Linear OpMode")
public final class RedBasket extends LinearOpMode {
    static int pos_multiplier = -1;
    static double botWidthHalf = 7.25;
    static double botLengthHalf = 7.5;
    static double firstSample_X = 38;
    static double firstSample_Y = 18.375;
    static double basket_X = 58.8674;
    static double basket_Y = 56.1745;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(pos_multiplier*(botWidthHalf), pos_multiplier*(-botLengthHalf+72), Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //Deposit Preload
                        .setTangent(Math.PI/2)
                        .lineToY(pos_multiplier*(19+botLengthHalf))
                        //Get First Sample
                        .lineToY(pos_multiplier*(27+3.75))
                        .splineToLinearHeading(new Pose2d(pos_multiplier*firstSample_X,pos_multiplier*firstSample_Y,Math.PI), Math.PI/2)
                        //Go to Basket
                        .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,5*Math.PI/4), Math.PI)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*(firstSample_X+10),pos_multiplier*firstSample_Y,Math.PI), Math.PI/2)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,5*Math.PI/4), Math.PI)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*48,pos_multiplier*48,Math.PI), Math.PI)

                        .splineToLinearHeading(new Pose2d(pos_multiplier*(firstSample_X+20),pos_multiplier*firstSample_Y,Math.PI), Math.PI/2)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*basket_X,pos_multiplier*basket_Y,5*Math.PI/4), Math.PI)
                        .splineToLinearHeading(new Pose2d(pos_multiplier*19,pos_multiplier*10,0), -Math.PI/2)


                        .build());
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    }
}
