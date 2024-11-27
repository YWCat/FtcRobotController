package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
    public static turnWrist turnWristAction = null;
    public static final double INTAKE_POS_CLAW = -1;
    public static final double OUTTAKE_POS_CLAW = 1;
    public static final double INTAKE_POWER_ROLLER = -1;
    public static final double OUTTAKE_POWER_ROLLER = 1;

    public enum Mode {
        ROLLER, CLAW
    }
    Mode mode = Mode.ROLLER;

    public static final double distanceThreshold = 2; //in inches
    public static final double WRIST_INTAKE_CLAW = 0.67;
    public static final double WRIST_OUTTAKE_CLAW = 0.1;
    public static final double WRIST_INTAKE_ROLLER = 0.7;
    public static final double WRIST_OUTTAKE_ROLLER = 0.2;
    public static final double WRIST_IDLE_ROLLER = 0.9;

    public SampleIntake(){
        RobotCore robotCore = RobotCore.getRobotCore();
        intakeServo = robotCore.hardwareMap.crservo.get(RobotConfig.sampleServo);
        distSensor = robotCore.hardwareMap.get(DistanceSensor.class, RobotConfig.distanceSensor);
        wristServo = robotCore.hardwareMap.get(Servo.class, RobotConfig.wrist); //0.5 is intake, 0.9 is outtake, base pos = 1.0, hold pos = 0.8
        intakeServo.setPower(0);
        wristServo.setPosition(WRIST_IDLE_ROLLER);
    }
    public double getWristPosition(){
        return wristServo.getPosition();
    }

    public final class startRoller implements Action {
        private boolean cancelled = false;
        private long startTime;
        private double servoPower; //
        private boolean isIntake;
        private boolean runStarted;
        private long timeout = 1000; //in milliseconds
        private DcMotorSimple.Direction direction;
        public startRoller(boolean intake){
            changeTarget(intake);
            Log.i(" Arm RobotActions", "Created new action startRoller");
        }
        public void changeTarget(boolean intake){
            if(mode == Mode.CLAW){
                intakeServo.setPower(0);
                direction = DcMotorSimple.Direction.FORWARD;
                if(intake){
                    direction = DcMotorSimple.Direction.FORWARD;
                    servoPower = INTAKE_POS_CLAW;
                    isIntake = true;
                } else{
                    servoPower = OUTTAKE_POS_CLAW;
                    isIntake = false;
                }
            } else {
                //intakeServo.setPower(0);
                direction = DcMotorSimple.Direction.FORWARD;
                if(intake){
                    servoPower = INTAKE_POWER_ROLLER;
                    isIntake = true;
                } else{
                    servoPower = OUTTAKE_POWER_ROLLER;
                    isIntake = false;
                }
            }
            runStarted = false;
            cancelled = false;

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
                Log.i("intakeServo RobotActions", "is cancelled?"  + cancelled);
                if(System.currentTimeMillis()-startTime < timeout && !cancelled) { //not timed out or forced to stop
                    if(mode == Mode.ROLLER && isIntake && distSensor.getDistance(DistanceUnit.INCH)<distanceThreshold){
                        Log.i("intakeServo RobotActions", "detected a sample");
                        intakeServo.setPower(0);
                        return false;
                    }
                    Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    return true;
                } else {
                    Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    if(mode == Mode.ROLLER){
                        intakeServo.setPower(0);
                    }
                    return false;
                }
            }
        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            //intakeServo.setPower(0);
            cancelled = true;
            Log.i("intakeServo RobotActions", "action cancelled");
        }
    }

    public class turnWrist implements Action{
        private boolean cancelled = false;
        private long startTime;
        private double wristPos;
        private boolean runStarted;
        private long timeout = 100; //in milliseconds
        private DcMotorSimple.Direction direction;
        public turnWrist(double position){
            changeTarget(position);
            Log.i(" Arm RobotActions", "Created new action startRoller");
        }
        public void changeTarget(double position){
            wristPos = position;
            runStarted = false;
            cancelled = false;
            if(mode == Mode.ROLLER){
                timeout = 100000;
            } else{
                timeout = 100;
            }
        }

        public boolean run (@NonNull TelemetryPacket p){
            if(!runStarted){
                wristServo.setPosition(wristPos);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                Log.i("intakeServo RobotActions", "is cancelled?"  + cancelled);
                if(System.currentTimeMillis()-startTime < timeout && !cancelled) { //not timed out or forced to stop
                    Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    return true;
                } else {
                    Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
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
    public startRoller getStartRollerAction(boolean intake, boolean forceNew){
        if(!forceNew){
            if(startRollerAction == null){
                startRollerAction = new startRoller(intake);
            } else {
                startRollerAction.changeTarget(intake);
            }
            return startRollerAction;
        } else{
            return new startRoller(intake);
        }
    }

    public turnWrist getTurnWristAction(double position, boolean forceNew){
        if(!forceNew){
            if(turnWristAction == null){
                turnWristAction = new turnWrist(position);
            } else {
                turnWristAction.changeTarget(position);
            }
            return turnWristAction;
        } else{
            return new turnWrist(position);
        }
    }

    public Action getPrepIOAction(boolean intake, boolean forceNew){
        double wristPos = WRIST_IDLE_ROLLER;

        if(mode == Mode.CLAW){
            if(intake){
                wristPos = WRIST_OUTTAKE_CLAW;
            } else{
                wristPos = WRIST_INTAKE_CLAW;
            }
            return new ParallelAction(getTurnWristAction(wristPos, forceNew), getStartRollerAction(intake, forceNew));
        } else{
            if(intake){
                wristPos = WRIST_INTAKE_ROLLER;
            } else{
                wristPos = WRIST_OUTTAKE_ROLLER;
            }
            return new ParallelAction(getTurnWristAction(wristPos, forceNew));
        }
    }


}
