package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//remove @Disabled to see the program on the driver station
@Disabled
//This is a test program for testing the servo positions of the outtake servo
public class ArmTestOpMode extends OpMode {
    private double clawOpenPos = 0.5;//targetpos2
    private double clawClosePos = 0.8;//good
    private double armUpPos = 0.55;//targetpos1
    private double armDwnPos = 0.05;//good
    private Servo armServo = null;
    private Servo clawServo = null;
    private double targetPos1 = 0.55;
    private double targetPos2 = 0.5;
    boolean prev_dpad_up1;
    boolean prev_dpad_down1;
    boolean prev_dpad_up2;
    boolean prev_dpad_down2;

    @Override
    public void init() {
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }
    @Override
    public void loop() {
        if(gamepad1.dpad_up && !prev_dpad_up1){
            targetPos1 += 0.05;
            telemetry.addLine("dpad_up pressed");
        }
        if(gamepad1.dpad_down && !prev_dpad_down1){
            targetPos1 -= 0.05;
            telemetry.addLine("dpad_down pressed");
        }
        prev_dpad_up1 = gamepad1.dpad_up;
        prev_dpad_down1 = gamepad1.dpad_down;
        //armServo is constantly going to targetPos1
        armServo.setPosition(targetPos1);
        //tells you targetPos
        telemetry.addData("Arm Target Position: ", targetPos1);
        //tells you the current position
        telemetry.addData("Arm Position: ", armServo.getPosition());



        if(gamepad2.dpad_up && !prev_dpad_up2){
            targetPos2 += 0.05;
            telemetry.addLine("dpad_up pressed");
        }
        if(gamepad2.dpad_down && !prev_dpad_down2){
            targetPos2 -= 0.05;
            telemetry.addLine("dpad_down pressed");
        }
        prev_dpad_up2 = gamepad2.dpad_up;
        prev_dpad_down2 = gamepad2.dpad_down;
        //armServo is constantly going to targetPos1
        clawServo.setPosition(targetPos2);
        //tells you targetPos
        telemetry.addData("Claw Target Position: ", targetPos2);
        //tells you the current position
        telemetry.addData("Claw Position: ", clawServo.getPosition());
        telemetry.update();
    }
}
