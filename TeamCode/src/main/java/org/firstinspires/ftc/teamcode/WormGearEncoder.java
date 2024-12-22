package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="WormGearEncoder", group="Linear OpMode")
public class WormGearEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx wormMotor = hardwareMap.get(DcMotorEx.class, "motor");
        waitForStart();
        while (!isStopRequested()) {
            wormMotor.getCurrentPosition();
            telemetry.addData("Worm Motor Encoder Reading: ", wormMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
