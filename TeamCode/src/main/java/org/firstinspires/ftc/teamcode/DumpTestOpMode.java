package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//remove @Disabled to see the program on the driver station
@Disabled
//This is a test program for testing the servo positions of the outtake servo
public class DumpTestOpMode extends OpMode {
    //our position for closing the outtake
    private double targetPos = 0.355;
    //our servo
    private Servo dumpServo;
    boolean prev_dpad_up;
    boolean prev_dpad_down;

    @Override
    public void init() {
        //we take in dumpServo based on the hardware map
        dumpServo = hardwareMap.get(Servo.class, "dumpServo");
    }
    @Override
    public void loop() {
        //press dpad_up to adjust the servo towards the close position
        if(gamepad1.dpad_up && !prev_dpad_up){
            targetPos += 0.05;
            telemetry.addLine("dpad_up pressed");
        }
        //press dpad_down to adjust the servo towards the open position
        if(gamepad1.dpad_down && !prev_dpad_down){
            targetPos -= 0.05;
            telemetry.addLine("dpad_down pressed");
        }
        prev_dpad_up = gamepad1.dpad_up;
        prev_dpad_down = gamepad1.dpad_down;
        //dumpServo constantly goes to the targetPos
        dumpServo.setPosition(targetPos);
        //tells you targetPos
        telemetry.addData("Target Position: ", targetPos);
        //tells you the current position
        telemetry.addData("Position: ", dumpServo.getPosition());
        telemetry.update();
    }
}
