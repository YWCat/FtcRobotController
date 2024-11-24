package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

public class SampleIntake {
    CRServo intakeServo;
    DistanceSensor distSensor;
    Servo wristServo;
    public static startRoller startRollerAction = null;
    public static final double ROLLER_POWER = 1;
    public static final double distanceThreshold = 2; //in inches
    public static final double WRIST_INTAKE = 1;
    public static final double WRIST_OUTTAKE = 1;
    public static final double WRIST_IDLE = 0.5;

    public SampleIntake(){
        RobotCore robotCore = RobotCore.getRobotCore();
        intakeServo = robotCore.hardwareMap.crservo.get(RobotConfig.sampleServo);
        distSensor = robotCore.hardwareMap.get(DistanceSensor.class, RobotConfig.distanceSensor);
        wristServo = robotCore.hardwareMap.get(Servo.class, RobotConfig.wrist); //0.5 is intake, 0.9 is outtake, base pos = 1.0, hold pos = 0.8

    }

    public final class startRoller implements Action {
        private boolean cancelled = false;
        private long startTime;
        private double servoPower; //
        private boolean isIntake;
        private double wristPos;
        private boolean runStarted;
        private long timeout = 10000; //in milliseconds
        private DcMotorSimple.Direction direction;
        public startRoller(double power){
            changeTarget(power);
            Log.i(" Arm RobotActions", "Created new action startRoller");
        }
        public void changeTarget(double power){
            servoPower = power;
            //servoPower = Math.abs(servoPower);
            intakeServo.setPower(0);
            direction = DcMotorSimple.Direction.FORWARD;
            if(power >= 0){
                direction = DcMotorSimple.Direction.FORWARD;
                wristPos = WRIST_INTAKE;
                isIntake = true;
            } else{
                //direction = DcMotorSimple.Direction.REVERSE;
                wristPos = WRIST_OUTTAKE;
                isIntake = false;
            }
            runStarted = false;
            cancelled = false;
        }

        public boolean run (@NonNull TelemetryPacket p){
            if(!runStarted){
                intakeServo.setPower(servoPower);
                intakeServo.setDirection(direction);
                wristServo.setPosition(wristPos);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                Log.i("intakeServo RobotActions", "is cancelled?"  + cancelled);
                if(System.currentTimeMillis()-startTime < timeout && !cancelled) { //not timed out or forced to stop
                    if(isIntake && distSensor.getDistance(DistanceUnit.INCH)<distanceThreshold){
                        Log.i("intakeServo RobotActions", "detected a sample");
                        intakeServo.setPower(0);
                        wristServo.setPosition(WRIST_IDLE);
                        return false;
                    }
                    Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    return true;
                } else {
                    Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    intakeServo.setPower(0);
                    wristServo.setPosition(WRIST_IDLE);
                    return false;
                }
            }
        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            intakeServo.setPower(0);
            cancelled = true;
            Log.i("intakeServo RobotActions", "action cancelled");
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



}
