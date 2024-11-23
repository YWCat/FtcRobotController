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
        double servoPos = 0;
        Servo wristServo = hardwareMap.get(Servo.class, "servo2"); //
        Servo specServo = hardwareMap.get(Servo.class, "servo4");//1.0 for grab, 0.7 for release
        CRServo sampleServo = hardwareMap.get(CRServo.class, "servo5");//negative is intake, positive is outtake
        waitForStart();
        while (!isStopRequested()) {
            if(gamepad1.dpad_right){
                servoPos += 0.1;
            }
            if(gamepad1.dpad_left){
                servoPos -= 0.1;
            }
            wristServo.setPosition(servoPos);
            telemetry.addData("Servo pos= ",servoPos);
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