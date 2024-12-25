package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

public class Arm {
    private DcMotorEx armMotor;
    private RotatingSlide rotatingSlide;
    public armToPosition prevMoveArmAction = null;
    private static final double TICKS_PER_REV = 384.5; //
    private static final int TARGET_TOLERANCE = 70; //units: ticks
        //approx the height of a sample when doing intake
    private static final double UP_VELOCITY = 2500; // x inches per 1 second // highly doubtful

    private static final double HOLD_POWER = 0;

    /*
    default pid:
    p = 10
    i = 0.05
    d = 0
    f = 0
    alg = LegacyPID
     */

    /*
    Refers to the arm that rotates the slides :)
     */

    public Arm(RotatingSlide rotatingSlide){
        this.rotatingSlide = rotatingSlide;
        RobotCore robotCore = RobotCore.getRobotCore();
        armMotor = robotCore.hardwareMap.get(DcMotorEx.class, RobotConfig.armMotor);
        /*armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         */ //Do not reset encoder
        armMotor.setTargetPositionTolerance(TARGET_TOLERANCE);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //armMotor.setPositionPIDFCoefficients(5);
        //PIDFCoefficients customPID = new PIDFCoefficients(10, 0.05, 0, 0);
        //armMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, customPID);
    }
    public void resetEncoder(){
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void stopMotor(){
        //slideMotorL.setVelocity(0.0);
        //slideMotorR.setVelocity(0.0);
        armMotor.setPower(HOLD_POWER);
    }

    public final class armToPosition implements Action {
        private long startTime;
        private boolean cancelled;
        private int targetPosition;
        private double factor;
        private boolean runStarted;
        private long timeout = 10000;
        public armToPosition(int pos,  double factor){
            changeTarget(pos, factor);
            Log.i(" Arm RobotActions", "Created new action armToPosition.");
        }
        public void changeTarget(int pos, double factor){
            targetPosition = pos;
            runStarted = false;
            cancelled = false;
            this.factor = factor;
            Log.i("armMotor changeTarget() called", "motor pos: " + armMotor.getCurrentPosition() + " target pos: " + targetPosition + "power factor: " +factor);
        }
        public boolean run(@NonNull TelemetryPacket p){
            if(!runStarted){
                if(rotatingSlide.getSlideExceedsHorizontalLimit() && targetPosition>armMotor.getCurrentPosition()){
                    armMotor.setVelocity(0);
                    Log.i("arm Horizontal Limit","Horizontal limit exceeded, stopping action before start");
                    return false;
                }
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armMotor.setVelocity(UP_VELOCITY*factor);
                startTime = System.currentTimeMillis();
                runStarted = true;
                Log.i("arm pid", "" + armMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
                Log.i("armMotor", "start running action with target position = " + targetPosition );

                return true;
            }
            else{
                boolean motorBusy = armMotor.isBusy();
                int currentPosition =armMotor.getCurrentPosition();
                double currentVelocity = armMotor.getVelocity();
                boolean targetReached = (Math.abs(currentPosition - targetPosition) <= TARGET_TOLERANCE) && (currentVelocity <= 20);
                boolean timeOut = (System.currentTimeMillis()-startTime >= timeout);
                Log.i("armMotor RobotActions", "motor pos: " + currentPosition + "target pos: " + targetPosition + "velocity: " + currentVelocity + "reached? = " + targetReached);
                //if(motorBusy && !timeOut && !cancelled) {
                if (!targetReached && !timeOut && !cancelled ) {
                    Log.i("armMotor RobotActions", "target not reached and not cancelled");
                    /*
                    if horizontal limit exceeded, set vel to zero and cancel action
                     */
                    if(rotatingSlide.getSlideExceedsHorizontalLimit()  && targetPosition>armMotor.getCurrentPosition()){
                        armMotor.setVelocity(0);
                        Log.i("arm Horizontal Limit","Horizontal limit exceeded, stopping action");
                        return false;
                    }
                    return true;
                } else {
                    //if (!motorBusy){
                    if (targetReached) {
                        Log.i("armMotor RobotActions", "arm done lifting: " + armMotor.getCurrentPosition() + "target pos: " + armMotor.getTargetPosition());
                    } else if (timeOut){
                        Log.i("armMotor RobotActions", "timeout; startTime " + startTime + " Position: " + armMotor.getCurrentPosition() + "target pos: " + armMotor.getTargetPosition());
                    } else if (cancelled){
                        Log.i("armMotor RobotActions", "cancelled");
                    } else {
                        Log.i("armMotor RobotActions", "stopped no reason");
                    }
                    armMotor.setVelocity(0);
                    armMotor.setPower(HOLD_POWER);
                    Log.i("AUTO", "Done arm: target = "+targetPosition);
                    return false;
                }
            }
        }

        public void cancel(){
            armMotor.setVelocity(0);
            armMotor.setPower(HOLD_POWER);
            cancelled = true;
            Log.i("armMotor RobotActions", "arm movement cancelled: " + armMotor.getCurrentPosition() + "target pos: " + armMotor.getTargetPosition());
        }
    }

    public armToPosition getArmToPosition(int pos, double factor, boolean forceNew){
        if(!forceNew){
            if(prevMoveArmAction == null){
                prevMoveArmAction = new armToPosition(pos, factor);
            } else {
                prevMoveArmAction.changeTarget(pos, factor);
            }
            return prevMoveArmAction;
        } else{
            return new armToPosition(pos, factor);
        }
    }
    public armToPosition getArmToPosition(double angle, double factor, boolean forceNew){
        return getArmToPosition(angleToTicks(angle),  factor,  forceNew);
    }
    public armToPosition getArmToPosition(double angle, boolean forceNew){
        return getArmToPosition(angleToTicks(angle),  1,  forceNew);
    }
    public armToPosition getArmToPosition(int position, boolean forceNew){
        return getArmToPosition(position, 1,  forceNew);
    }

    public int getMotorPositionTicks(){
        return armMotor.getCurrentPosition();
    }
    public double getMotorPositionAngle(){
        return ticksToAngle(armMotor.getCurrentPosition());
    }


    public double ticksToAngle(int ticks){
        return ((double) ticks / TICKS_PER_REV / 28 * 360); //28 = worm gear gear ratio
    } // 1 degree is around 30 ticks, 90 degrees is 2690 something.

    public int angleToTicks(double angle){
        return (int) (angle / 360 * 28 * TICKS_PER_REV); //28 = worm gear gear ratio
    }
}
