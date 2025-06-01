package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideSubsystem{
    //if the error is too high, increase Kp a little
    private double Kp = 0.007;
    private static final double TICKS_PER_REV = 384.5; //5204-8002-0014, 435 RPM
    private static final double PULLEY_DIAMETER_IN = (32 / 25.4); //3D printed part // = 1.25984251969 inches
    private DcMotor slideMotorL;
    private DcMotor slideMotorR;

    //The constructor, takes in hardwareMap and initializes both motors
    public SlideSubsystem(HardwareMap hardwareMap){
        slideMotorL = hardwareMap.get(DcMotor.class, "slideMotorL");
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR = hardwareMap.get(DcMotor.class, "slideMotorR");
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void raiseSlide(double targetPos){
        //we have a limit for how high targetPos can get
        if(targetPos > 2810){
            targetPos = 2810;
        }
        //ditto but for lower limit
        if(targetPos < 0){
            targetPos = 0;
        }
        //rudimentary PID for getting the slide motors to the correct position
        double output1 = Kp * (targetPos - slideMotorL.getCurrentPosition());
        double output2 = Kp * (targetPos - slideMotorR.getCurrentPosition());
        slideMotorL.setPower(output1);
        slideMotorR.setPower(output2);
    }
}
