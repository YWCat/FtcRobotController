package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

@Config
public class DualMotorSlide {
    private DcMotorEx slideMotorL;
    private DcMotorEx slideMotorR;
    public static slideToPosition prevMoveSlideAction = null;
    private static final double TICKS_PER_REV = 751.8; //5203-2402-0027, 223 RPM
    private static final double PULLEY_DIAMETER_IN = (32.25 / 24.5); //3407-0002-0112 // = 1.269685 inches
    private final int TARGET_TOLERANCE = (int) (0.3*TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    private Telemetry telemetry;
    private boolean targetReached = true;
    public final double LEFT_SYNC_FACTOR = 1;
    private final double FAST_POWER = 0.6;
    private final double SLOW_POWER = 0.3;
    public static double SLIDE_HOLD_POWER = 0.03;
    private VoltageSensor batteryVoltageSensor;

    public static double  UP_VELOCITY = 500; // revolutions per second

    public DualMotorSlide (){
        // Set up Left and Right motors
        RobotCore robotCore = RobotCore.getRobotCore();
        slideMotorL = robotCore.hardwareMap.get(DcMotorEx.class, RobotConfig.slideMotorL);
        slideMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMotorL.setTargetPositionTolerance(TARGET_TOLERANCE);

        slideMotorR = robotCore.hardwareMap.get(DcMotorEx.class, RobotConfig.slideMotorR);
        slideMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMotorR.setTargetPositionTolerance(TARGET_TOLERANCE);

        /*Reverse the motors if necessary*/
        //slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        //slideMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public final class slideToPosition implements Action {
        private long startTime;
        private int targetPosition; //ticks
        private boolean runStarted;
        private long timeout = 10000;
        public slideToPosition(int pos){
            changeTarget(pos);
            Log.i(" Slide RobotActions", "Created new action");
        }
        public void changeTarget(int pos){
            targetPosition = pos;
            runStarted = false;
        }
        public boolean run(@NonNull TelemetryPacket p){
            if(!runStarted){
                slideMotorL.setTargetPosition(targetPosition);
                slideMotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slideMotorL.setVelocity(UP_VELOCITY);
                //Adjusts the speed of slideMotorR to follow slideMotorL
                slideMotorR.setVelocity(UP_VELOCITY*LEFT_SYNC_FACTOR);

                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                if(slideMotorL.isBusy() && System.currentTimeMillis()-startTime < timeout) {
                    Log.i("slideMotor RobotActions", "motor pos: " + slideMotorL.getCurrentPosition() + "target pos: " + slideMotorL.getTargetPosition());
                    return true;
                } else {
                    if (!slideMotorL.isBusy()){
                        Log.i("slideMotor RobotActions", "slide done moving: " + slideMotorL.getCurrentPosition() + "target pos: " + slideMotorL.getTargetPosition());
                    } else{
                        Log.i("slideMotor RobotActions", "timeout; position: " + slideMotorL.getCurrentPosition() + "target pos: " + slideMotorL.getTargetPosition());
                    }
                    slideMotorL.setVelocity(0.0);
                    slideMotorR.setVelocity(0.0);
                    slideMotorL.setPower(SLIDE_HOLD_POWER);
                    slideMotorR.setPower(SLIDE_HOLD_POWER);
                    return false;
                }
            }
        }
    }
    public slideToPosition getSlideToPosition(int pos, boolean forceNew){
        if(!forceNew){
            if(prevMoveSlideAction == null){
                prevMoveSlideAction = new slideToPosition(pos);
            } else {
                prevMoveSlideAction.changeTarget(pos);
            }
            return prevMoveSlideAction;
        } else{
            return new slideToPosition(pos);
        }
    }

    public int getMotorLPosition(){
        return slideMotorL.getCurrentPosition();
    }

}
