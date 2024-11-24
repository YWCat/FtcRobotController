package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

public class Arm {
    DcMotorEx armMotor;
    public static armToPosition prevMoveArmAction = null;
    private static final double TICKS_PER_REV = -1; //
    private static final int TARGET_TOLERANCE = 100; //units: ticks
    private static final double UP_VELOCITY = 1000; // x inches per 1 second // highly doubtful

    private static final double HOLD_POWER = 0;

    /*
    Refers to the arm that rotates the slides :)
     */

    public Arm(){
        RobotCore robotCore = RobotCore.getRobotCore();
        armMotor = robotCore.hardwareMap.get(DcMotorEx.class, RobotConfig.armMotor);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPositionTolerance(TARGET_TOLERANCE);
    }
    public final class armToPosition implements Action {
        private long startTime;
        private boolean cancelled;
        private int targetPosition;
        private boolean runStarted;
        private long timeout = 10000;
        public armToPosition(int pos){
            changeTarget(pos);
            Log.i(" Arm RobotActions", "Created new action PreChamber");
        }
        public void changeTarget(int pos){
            targetPosition = pos;
            runStarted = false;
            cancelled = false;
        }
        public boolean run(@NonNull TelemetryPacket p){
            if(!runStarted){
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armMotor.setVelocity(UP_VELOCITY);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                if(armMotor.isBusy() && System.currentTimeMillis()-startTime < timeout && !cancelled) {
                    Log.i("armMotor RobotActions", "motor pos: " + armMotor.getCurrentPosition() + "target pos: " + armMotor.getTargetPosition());
                    return true;
                } else {
                    if (!armMotor.isBusy()){
                        Log.i("armMotor RobotActions", "arm done lifting: " + armMotor.getCurrentPosition() + "target pos: " + armMotor.getTargetPosition());
                    } else{
                        Log.i("armMotor RobotActions", "timeout; position: " + armMotor.getCurrentPosition() + "target pos: " + armMotor.getTargetPosition());
                    }
                    armMotor.setVelocity(0);
                    armMotor.setPower(HOLD_POWER);
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

    public armToPosition getArmToPosition(int pos, boolean forceNew){
        if(!forceNew){
            if(prevMoveArmAction == null){
                prevMoveArmAction = new armToPosition(pos);
            } else {
                prevMoveArmAction.changeTarget(pos);
            }
            return prevMoveArmAction;
        } else{
            return new armToPosition(pos);
        }
    }

    public int getMotorPosition(){
        return armMotor.getCurrentPosition();
    }
}
