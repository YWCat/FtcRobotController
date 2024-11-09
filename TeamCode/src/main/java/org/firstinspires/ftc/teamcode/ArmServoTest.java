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
    private double intakePos0 = 0.5, intakePos1 = 0, intakePos2 = 0;
    private double outtakePos0 = 1.0, outtakePos1 = 0, outtakePos2 = 0;
    private double basePos0 = 0.5, basePos1 = 0, basePos2 = 0;
    public int intakeState = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo armServo = hardwareMap.get(Servo.class, "servo0"); //0.5 is intake, 0.9 is outtake, base pos = 1.0, hold pos = 0.8
        Servo wristServo = hardwareMap.get(Servo.class, "servo1");//0.35 is intake, 0 is outtake, base pos = 0.35, hold pos = 0
        CRServo sampleServo = hardwareMap.get(CRServo.class, "servo2");//negative is intake, positive is outtake
        DistanceSensor distSensor = hardwareMap.get(DistanceSensor.class, "distSample");
        waitForStart();
        while (!isStopRequested()) {
            switch(intakeState) {
                case 0://base state
                    armServo.setPosition(1.0);
                    wristServo.setPosition(0.35);
                    sampleServo.setPower(0);
                    if(gamepad1.a){
                        intakeState=1;
                    }
                    break;
                case 1://intake state
                    armServo.setPosition(0.5);
                    wristServo.setPosition(0.35);
                    sampleServo.setPower(-1.0);
                    telemetry.addData("Distance:", distSensor.getDistance(DistanceUnit.INCH));
                    if(distSensor.getDistance(DistanceUnit.INCH)<2){
                        intakeState=2;
                    }
                    break;
                case 2://hold state
                    armServo.setPosition(0.8);
                    wristServo.setPosition(0);
                    sampleServo.setPower(0);
                    if(gamepad1.left_bumper){
                        intakeState=3;
                    }
                    if(gamepad1.a){
                        intakeState=1;
                    }
                    break;
                case 3://outtake state
                    armServo.setPosition(0.9);
                    wristServo.setPosition(0);
                    sampleServo.setPower(1.0);
                    if(gamepad1.x){
                        intakeState=0;
                    }
                    break;
                default:
            }
            telemetry.update();
            sleep(100);
        }
    }
}