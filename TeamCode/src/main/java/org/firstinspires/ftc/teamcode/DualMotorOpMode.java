package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
//This is a test program for determining the ideal height of the slides
public class DualMotorOpMode extends OpMode{
    //variable we adjust for the slide to go up or down
    private double targetPos = 0;
    //constant that can be adjusted to change the PID
    private double Kp = 0.007;
    private DcMotor slideMotorL;
    private DcMotor slideMotorR;

    private static final double TICKS_PER_REV = 384.5; //5204-8002-0014, 435 RPM
    private static final double PULLEY_DIAMETER_IN = (32 / 25.4); //3D printed part // = 1.25984251969 inches
    public int inchToTicks(double inches) {
        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    }
    boolean prev_dpad_up;
    boolean prev_dpad_down;

    @Override
    public void init() {
        //initializes both motors
	    slideMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    slideMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        // linearSlide1.setTargetPosition(targetPos);
        // linearSlide2.setTargetPosition(targetPos);
        //Press dpad_up to go up(you will have to spam the button)
        if(gamepad1.dpad_up && !prev_dpad_up){
            targetPos += inchToTicks(1.0);
        }
        //Press dpad_down to go down(you will have to spam the button)
        if(gamepad1.dpad_down && !prev_dpad_down){
            targetPos -= inchToTicks(1.0);
        }
        //lower limit for targetPos
        if(targetPos<0){
            targetPos = 0;
        }
        //makes sure that holding down dpad_up or dpad_down will not rapidly change the targetPos
        prev_dpad_up = gamepad1.dpad_up;
        prev_dpad_down = gamepad1.dpad_down;
        //rudimentary PID for determining slide power
        double output1 = Kp * (targetPos - slideMotorL.getCurrentPosition());
        double output2 = Kp * (targetPos - slideMotorR.getCurrentPosition());
        //slides get set to that power
        slideMotorL.setPower(output1);
        slideMotorR.setPower(output2);
        //telemetry.addData("Error_L:", Math.abs(slideMotorL.getCurrentPosition()-targetPos));
        //telemetry.addData("Error_R:", Math.abs(slideMotorR.getCurrentPosition()-targetPos));
        //The current position of both of the slide motors
        telemetry.addData("Left Motor: ", slideMotorL.getCurrentPosition());
        telemetry.addData("Right Motor: ", slideMotorR.getCurrentPosition());
        telemetry.update();
    }
}
