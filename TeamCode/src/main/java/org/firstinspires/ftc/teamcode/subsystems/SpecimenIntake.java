package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

public class SpecimenIntake {

    Servo specimenServo;
    Servo specimenWrist;
    public static moveSpecimenIntake prevMoveSpecimen = null;
    public static moveSpecimenWrist prevMoveWrist = null;
    public static final double INTAKE_WRIST = 0.825;
    public static final double OUTTAKE_WRIST = 0.22;
    public static final double OPEN = 0.35;
    public static final double CLOSE = 0.65;
    public SpecimenIntake(){
        RobotCore robotCore = RobotCore.getRobotCore();
        specimenServo = robotCore.hardwareMap.get(Servo.class, RobotConfig.specimenServo);
        specimenWrist = robotCore.hardwareMap.get(Servo.class, RobotConfig.specimenWrist);
        specimenServo.setPosition(CLOSE);
        specimenWrist.setPosition(OUTTAKE_WRIST);
    }

    public boolean isClawOpen(){
        return specimenServo.getPosition() < (OPEN+CLOSE)/2; //assuming open is closer to 0 than close
    }

    public boolean isWristAtIntakePosition(){
        return specimenWrist.getPosition() > (INTAKE_WRIST+OUTTAKE_WRIST)/2; //assuming intake is closer to 1 than outtake
    }
    public final class moveSpecimenIntake implements Action {
        private long startTime;
        private double targetPosition;

        private boolean runStarted;
        private long timeout = 500;
        public moveSpecimenIntake(double pos){
            changeTarget(pos);
            //Log.i(" Arm RobotActions", "Created new action PreChamber");
        }
        public void changeTarget(double pos){
            targetPosition = pos;
            runStarted = false;
        }

        public boolean run(@NonNull TelemetryPacket p){
            if(!runStarted){
                specimenServo.setPosition(targetPosition);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                if(System.currentTimeMillis()-startTime < timeout) {
                    //Log.i("specimenServo RobotActions", "target pos: " + specimenServo.getPosition());
                    return true;
                } else {
                    //Log.i("specimenServo RobotActions", "target pos: " + specimenServo.getPosition());
                    return false;
                }
            }
        }

    }

    public final class moveSpecimenWrist implements Action {
        private long startTime;
        private double targetPosition;

        private boolean runStarted;
        private long timeout = 500;
        public moveSpecimenWrist(double pos){
            changeTarget(pos);
            //Log.i(" Arm RobotActions", "Created new action PreChamber");
        }
        public void changeTarget(double pos){
            targetPosition = pos;
            runStarted = false;
        }

        public boolean run(@NonNull TelemetryPacket p){
            if(!runStarted){
                specimenWrist.setPosition(targetPosition);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                if(System.currentTimeMillis()-startTime < timeout) {
                    //Log.i("specimenWrist RobotActions", "target pos: " + specimenWrist.getPosition());
                    return true;
                } else {
                    //Log.i("specimenWrist RobotActions", "target pos: " + specimenWrist.getPosition());
                    return false;
                }
            }
        }

    }

    public moveSpecimenIntake getMoveSpecimenIntake(double pos, boolean forceNew){
        if(!forceNew){
            if(prevMoveSpecimen == null){
                prevMoveSpecimen = new moveSpecimenIntake(pos);
            } else {
                prevMoveSpecimen.changeTarget(pos);
            }
            return prevMoveSpecimen;
        } else{
            return new moveSpecimenIntake(pos);
        }
    }
    public moveSpecimenWrist getMoveSpecimenWrist(double pos, boolean forceNew){
        if(!forceNew){
            if(prevMoveWrist == null){
                prevMoveWrist = new moveSpecimenWrist(pos);
            } else {
                prevMoveWrist.changeTarget(pos);
            }
            return prevMoveWrist;
        } else{
            return new moveSpecimenWrist(pos);
        }
    }

}
