package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="ArmServoTest", group="Linear OpMode")
public final class ArmServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double armPos = 0.5;
        double claw2Pos = 0.9;
        Servo arm = hardwareMap.get(Servo.class, "sampleServo"); //
        Servo claw = hardwareMap.get(Servo.class, "wristServo");//1.0 for grab, 0.7 for release
        waitForStart();
        while (!isStopRequested()) {
            if(gamepad1.dpad_right){
                //claw2Pos += 0.1;
                claw2Pos = 1.0;
            }
            if(gamepad1.dpad_left){
                //claw2Pos -= 0.1;
                claw2Pos = -0.1;
            }
            if(gamepad1.dpad_up){
                armPos += 0.005;
            }
            if(gamepad1.dpad_down){
                armPos -= 0.005;
            }
            arm.setPosition(armPos);
            claw.setPosition(claw2Pos);
            telemetry.addData("claw pos= ",claw2Pos);
            telemetry.addData("arm pos= ",armPos);
            /*
            if(gamepad1.dpad_left){
                sampleServo.setPower(-1.0);
                telemetry.addLine("left pressed");
            }else if(gamepad1.dpad_right){
                sampleServo.setPower(1.0);
                telemetry.addLine("right pressed");
            }
            else {
                sampleServo.setPower(0);
                telemetry.addLine("nothing is pressed");
            }
            */
            telemetry.update();
            telemetry.setMsTransmissionInterval(11);
            sleep(200);
        }
    }
}