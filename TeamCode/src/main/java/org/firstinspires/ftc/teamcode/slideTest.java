package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="slideTest", group="Linear OpMode")
public class slideTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx Lslide = hardwareMap.get(DcMotorEx.class,"left_motor");
        DcMotorEx Rslide = hardwareMap.get(DcMotorEx.class,"right_motor");
        Rslide.setDirection(DcMotorSimple.Direction.REVERSE);
        while (!isStopRequested()) {
            if (gamepad1.dpad_up) {
                Lslide.setPower(1);
                Rslide.setPower(1);
                telemetry.addLine("dpad up pressed");
            } else if (gamepad1.dpad_down) {
                Lslide.setPower(-1);
                Rslide.setPower(-1);
                telemetry.addLine("dpad down pressed");
            } else{
                Lslide.setPower(0);
                Rslide.setPower(0);
                telemetry.addLine("nothing pressed");
            }
            telemetry.update();
        }
    }
}
