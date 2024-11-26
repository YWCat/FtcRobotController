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
    public static moveSpecimenIntake prevMoveSpecimen = null;
    public static final double OPEN = 0.35;
    public static final double CLOSE = 0.65;
    public SpecimenIntake(){
        RobotCore robotCore = RobotCore.getRobotCore();
        specimenServo = robotCore.hardwareMap.get(Servo.class, RobotConfig.specimenServo);
        specimenServo.setPosition(CLOSE);
    }
    public final class moveSpecimenIntake implements Action {
        private long startTime;
        private double targetPosition;

        private boolean runStarted;
        private long timeout = 500;
        public moveSpecimenIntake(double pos){
            changeTarget(pos);
            Log.i(" Arm RobotActions", "Created new action PreChamber");
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
                    Log.i("specimenServo RobotActions", "target pos: " + specimenServo.getPosition());
                    return true;
                } else {
                    Log.i("specimenServo RobotActions", "target pos: " + specimenServo.getPosition());
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

}
