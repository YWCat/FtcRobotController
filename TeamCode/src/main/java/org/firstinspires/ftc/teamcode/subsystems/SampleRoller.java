package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

public class SampleRoller {
    CRServo intakeServo;
    public static startRoller startRollerAction = null;
    public static final double INTAKE_POWER = 1;
    public static final double OUTTAKE_POWER = 1;

    public SampleRoller(){
        RobotCore robotCore = RobotCore.getRobotCore();
        intakeServo = robotCore.hardwareMap.crservo.get(RobotConfig.claw);
    }

    public final class startRoller implements Action {
        private long startTime;
        private double servoPower; // §§
        private boolean runStarted;
        private long timeout = 10000; //in milliseconds
        private DcMotorSimple.Direction direction;
        public startRoller(double power){
            changeTarget(power);
            Log.i(" Arm RobotActions", "Created new action PreChamber");
        }
        public void changeTarget(double power){
            servoPower = Math.abs(power);
            if(power >= 0){
                direction = DcMotorSimple.Direction.FORWARD;
            } else{
                direction = DcMotorSimple.Direction.REVERSE;
            }
            runStarted = false;
        }

        public boolean run (@NonNull TelemetryPacket p){
            if(!runStarted){
                intakeServo.setPower(servoPower);
                intakeServo.setDirection(direction);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                if(System.currentTimeMillis()-startTime < timeout) {
                    //TODO: Add check for distance sensor
                    Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    return true;
                } else {
                    Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    intakeServo.setPower(0);
                    return false;
                }
            }
        }
    }

    public startRoller getStartRollerAction(double power, boolean forceNew){
        if(!forceNew){
            if(startRollerAction == null){
                startRollerAction = new startRoller(power);
            } else {
                startRollerAction.changeTarget(power);
            }
            return startRollerAction;
        } else{
            return new startRoller(power);
        }
    }

    public void stopActions(){ //call this AFTER clearing the actions list in LoopUpdater
        intakeServo.setPower(0);
    }

}
