package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="wormMotorEncoder", group="Linear OpMode")
public class wormMotorEncoder extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx wormMotor = hardwareMap.get(DcMotorEx.class,"worm_motor");
        int intakePos = 3984;
        int slidePos = 0;
        int tempPos;
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setTargetPosition(intakePos);
        wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (!isStopRequested()) {
            if(gamepad2.a){
                wormMotor.setTargetPosition(slidePos);
                wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormMotor.setPower(0.5);
            }
            if(gamepad2.b){
                wormMotor.setTargetPosition(intakePos);
                wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormMotor.setPower(0.5);
            }
            if(gamepad2.dpad_down){
                if(wormMotor.getCurrentPosition()-100 <= -200){
                    tempPos = -200;
                }
                else{
                    tempPos = wormMotor.getCurrentPosition()-100;
                }
                wormMotor.setTargetPosition(tempPos);
                wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormMotor.setPower(0.15);
            }
            if(gamepad2.dpad_up){
                if(wormMotor.getCurrentPosition()+100 <= 3984){
                    tempPos = 3984;
                }
                else{
                    tempPos = wormMotor.getCurrentPosition()+100;
                }
                wormMotor.setTargetPosition(wormMotor.getCurrentPosition()+100);
                wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormMotor.setPower(0.15);
            }
            telemetry.addData("Worm Motor Encoder Reading: ", wormMotor.getCurrentPosition());
            telemetry.setMsTransmissionInterval(11);
            telemetry.update();
        }
    }
}
