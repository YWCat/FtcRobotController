package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;
@Config
public class SampleIntake {
    CRServo  intakeServo;
    DistanceSensor distSensor;
    Servo wristServo;
    public static startAndStopRoller startAndStopRollerAction = null;
    public static startRoller startRollerAction = null;
    public static reverseRoller reverseRollerAction = null;
    public static waitAct waitAction = null;
    public static stopRoller stopRollerAction = null;
    public static turnWrist turnWristAction = null;
    public static  double INTAKE_POS_CLAW = -1;
    public static  double OUTTAKE_POS_CLAW = 1;
    public static  double INTAKE_POWER_ROLLER = -1;
    public static  double OUTTAKE_POWER_ROLLER = 0.5;

    public enum Mode {
        ROLLER, CLAW
    }
    Mode mode = Mode.ROLLER;

    public static  double distanceThreshold = 2; //in inches
    public static  double WRIST_INTAKE_CLAW = 0.67;
    public static  double WRIST_OUTTAKE_CLAW = 0.1;
    public static  double WRIST_INTAKE_ROLLER = 0.9; //0.88; 0.95(too high)
    public static  double WRIST_OUTTAKE_ROLLER = 0.12; //0.07;
    public static  double WRIST_IDLE_ROLLER = 0.9;
    public static  double WRIST_HANG_ROLLER = 0.0;
    public static  double WRIST_INIT_ROLLER = 0.0;
    public static  double WRIST_INIT_CLAW = 0.0;

    public SampleIntake(){
        RobotCore robotCore = RobotCore.getRobotCore();
        intakeServo = robotCore.hardwareMap.crservo.get(RobotConfig.sampleServo);
        distSensor = robotCore.hardwareMap.get(DistanceSensor.class, RobotConfig.distanceSensor);
        wristServo = robotCore.hardwareMap.get(Servo.class, RobotConfig.wrist); //0.5 is intake, 0.9 is outtake, base pos = 1.0, hold pos = 0.8
        intakeServo.setPower(0);
        //Log.v("intakeServo initialized", "power = " + 0);
        wristServo.setPosition(WRIST_INIT_ROLLER);
    }
    public double getWristPosition(){
        return wristServo.getPosition();
    }

    public void manualMoveRoller(double power){
        intakeServo.setPower(power);
        //Log.v("intakeServo manualMove", "power = " + power);
    }

    public final class startAndStopRoller implements Action {
        private boolean cancelled = false;
        private long startTime;
        private double servoPower; //
        private boolean isIntake;
        private boolean runStarted;
        private long timeout = 1500; //in milliseconds
        private DcMotorSimple.Direction direction;
        public startAndStopRoller(boolean intake){
            changeTarget(intake);
            //Log.i(" Arm RobotActions", "Created new action startAndStopRoller");
        }
        public void changeTarget(boolean intake){
            if(mode == Mode.CLAW){
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
                direction = DcMotorSimple.Direction.FORWARD;
                if(intake){
                    servoPower = INTAKE_POWER_ROLLER;
                    isIntake = true;
                } else{
                    servoPower = OUTTAKE_POWER_ROLLER;
                    isIntake = false;
                }
            }
            if(mode == Mode.ROLLER){

            } else{
                timeout = 100;
            }
            runStarted = false;
            cancelled = false;

        }

        public boolean run (@NonNull TelemetryPacket p){
            if(!runStarted){
                intakeServo.setPower(servoPower);
                //Log.i("intakeServo RobotActions", "set power: " + servoPower);
                intakeServo.setDirection(direction);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                //Log.i("intakeServo RobotActions", "is cancelled?"  + cancelled);
                if(System.currentTimeMillis()-startTime < timeout && !cancelled) { //not timed out or forced to stop
                    if(mode == Mode.ROLLER && isIntake && distSensor.getDistance(DistanceUnit.INCH)<distanceThreshold){
                        intakeServo.setPower(0);
                        //Log.i("intakeServo RobotActions", "detected a sample");
                        return false;
                    }
                    //Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    return true;
                } else {
                    //Log.i("intakeServo RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    if(mode == Mode.ROLLER){
                        intakeServo.setPower(0);
                        //Log.i("intakeServo RobotActions power off", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    }
                    return false;
                }
            }
        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            intakeServo.setPower(0);
            cancelled = true;
            //Log.i("intakeServo RobotActions", "action cancelled");
        }
    }
    public final class startRoller implements Action {
        private boolean cancelled = false;
        private long startTime;
        private double servoPower; //
        private boolean isIntake;
        private boolean runStarted;
        private long timeout = 1500; //in milliseconds
        private DcMotorSimple.Direction direction;
        public startRoller(boolean intake){
            changeTarget(intake);
            //Log.i(" intakeServoStartRoller RobotActions", "Created new action startAndStopRoller");
        }
        public void changeTarget(boolean intake){
            if(mode == Mode.CLAW){
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
                direction = DcMotorSimple.Direction.FORWARD;
                if(intake){
                    servoPower = INTAKE_POWER_ROLLER;
                    isIntake = true;
                } else{
                    servoPower = OUTTAKE_POWER_ROLLER;
                    isIntake = false;
                }
            }
            if(mode == Mode.ROLLER){

            } else{
                timeout = 100;
            }
            runStarted = false;
            cancelled = false;

        }

        public boolean run (@NonNull TelemetryPacket p){
                intakeServo.setPower(servoPower);
                //Log.i("intakeServoStartRoller RobotActions", "set power: " + servoPower);
                intakeServo.setDirection(direction);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return false;

        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            intakeServo.setPower(0);
            cancelled = true;
            //Log.i("intakeServoStartRoller RobotActions", "action cancelled");
        }
    }

    public final class reverseRoller implements Action {
        private boolean cancelled = false;
        private long startTime;
        private double servoPower; //
        private boolean runStarted;
        private long timeout = 1500; //in milliseconds
        private DcMotorSimple.Direction direction;
        public reverseRoller(long tout){
            changeTarget(tout);
            //Log.i(" intakeServoReverseRoller RobotActions", "Created new action reverseRoller");
        }
        public void changeTarget(long tout){
            if(mode == Mode.CLAW){
                direction = DcMotorSimple.Direction.FORWARD;
                //TODO
            } else {
                direction = DcMotorSimple.Direction.REVERSE;
                servoPower = OUTTAKE_POWER_ROLLER;
            }
            timeout = tout;
            runStarted = false;
            cancelled = false;

        }

        public boolean run (@NonNull TelemetryPacket p){
            if (!runStarted) {
                intakeServo.setPower(servoPower);
                //Log.i("intakeServoRvsRoller RobotActions", "REVERSE set power: " + servoPower + " timeout: " + timeout);
                intakeServo.setDirection(direction);
                startTime = System.currentTimeMillis();
                runStarted = true;
                Actions.runBlocking( new SleepAction(0.01));
                return true;
            } else if (System.currentTimeMillis() - startTime >= timeout) {
                intakeServo.setPower(servoPower);
                intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                Actions.runBlocking( new SleepAction(0.01));
                //Log.i("intakeServoRvsRoller RobotActions", "FORWARD set power: " + servoPower);
                return false;
            } else {
                Actions.runBlocking( new SleepAction(0.01));
                //Log.i("intakeServoRvsRoller RobotActions", "wait for timeout");
                return true;
            }

        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            intakeServo.setPower(0);
            cancelled = true;
            //Log.i("intakeServoStartRoller RobotActions", "action cancelled");
        }
    }

    public final class stopRoller implements Action {
        private boolean cancelled = false;
        private long startTime;
        private double servoPower; //
        private boolean isIntake;
        private boolean runStarted;
        private long timeout = 1500; //in milliseconds
        private DcMotorSimple.Direction direction;
        public stopRoller(){
            changeTarget();
            //Log.i(" intakeServo StopRoller RobotActions", "Created new action startAndStopRoller");
        }
        public void changeTarget(){
            if(mode == Mode.CLAW){
                direction = DcMotorSimple.Direction.FORWARD;
                servoPower = OUTTAKE_POS_CLAW;
            } else {
                direction = DcMotorSimple.Direction.FORWARD;
                servoPower = 0;
            }
            if(mode == Mode.ROLLER){

            } else{
                timeout = 100;
            }
            runStarted = false;
            cancelled = false;

        }

        public boolean run (@NonNull TelemetryPacket p){
            intakeServo.setPower(servoPower);
            //Log.i("intakeServo StopRoller RobotActions", "set power: " + servoPower);
            intakeServo.setDirection(direction);
            startTime = System.currentTimeMillis();
            runStarted = true;
            return false;

        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            intakeServo.setPower(0);
            cancelled = true;
            //Log.i("intakeServo StopRoller RobotActions", "action cancelled");
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
            //Log.i(" Arm RobotActions", "Created new action startAndStopRoller");
        }
        public void changeTarget(double position){
            wristPos = position;
            runStarted = false;
            cancelled = false;

        }

        public boolean run (@NonNull TelemetryPacket p){
            if(!runStarted){
                wristServo.setPosition(wristPos);
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            }
            else{
                //Log.i("intakeWrist RobotActions", "is cancelled?"  + cancelled);
                if(System.currentTimeMillis()-startTime < timeout && !cancelled) { //not timed out or forced to stop
                    //Log.i("intakeWrist RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    return true;
                } else {
                    //Log.i("intakeWrist RobotActions", "power: " + intakeServo.getPower() + "direction: " + intakeServo.getDirection());
                    return false;
                }
            }
        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            intakeServo.setPower(0);
            cancelled = true;
            //Log.i("intakeServo RobotActions", "action cancelled");
        }
    }
    public startAndStopRoller getStartAndStopRollerAction(boolean intake, boolean forceNew){
        if(!forceNew){
            if(startAndStopRollerAction == null){
                startAndStopRollerAction = new startAndStopRoller(intake);
            } else {
                startAndStopRollerAction.changeTarget(intake);
            }
            return startAndStopRollerAction;
        } else{
            return new startAndStopRoller(intake);
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

    public final class waitAct implements Action {
        private boolean cancelled = false;
        private long startTime;
        private boolean runStarted;
        private long timeout = 1500; //in milliseconds
        private long cnt = 0;
        public waitAct(long tout){
            changeTarget(tout);
            //Log.i(" intakeServoReverseRoller RobotActions", "Created new action reverseRoller");
        }
        public void changeTarget(long tout){
            timeout = tout;
            runStarted = false;
            cancelled = false;
        }

        public boolean run (@NonNull TelemetryPacket p){
            if (!runStarted) {
                startTime = System.currentTimeMillis();
                runStarted = true;
                cnt++;
                return true;
            } else if (System.currentTimeMillis() - startTime >= timeout) {
                Log.i("waitAct", "timeout: " + timeout);
                return false;
            } else {
                cnt++;
                Log.i("waitAct", "wait for timeout. cnt = " + cnt);
                return true;
            }

        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            intakeServo.setPower(0);
            cancelled = true;
            //Log.i("intakeServoStartRoller RobotActions", "action cancelled");
        }
    }

    public waitAct getWaitAction(long timeout, boolean forceNew){
        if(!forceNew){
            if(waitAction == null){
                waitAction = new waitAct(timeout);
            } else {
                waitAction.changeTarget(timeout);
            }
            return waitAction;
        } else{
            return new waitAct(timeout);
        }
    }

    public reverseRoller getReverseRollerAction(long timeout, boolean forceNew){
        if(!forceNew){
            if(reverseRollerAction == null){
                reverseRollerAction = new reverseRoller(timeout);
            } else {
                reverseRollerAction.changeTarget(timeout);
            }
            return reverseRollerAction;
        } else{
            return new reverseRoller(timeout);
        }
    }
    public stopRoller getStopRollerAction( boolean forceNew){
        if(!forceNew){
            if(stopRollerAction == null){
                stopRollerAction = new stopRoller();
            } else {
                stopRollerAction.changeTarget();
            }
            return stopRollerAction;
        } else{
            return new stopRoller();
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
            return new ParallelAction(getTurnWristAction(wristPos, forceNew), getStartAndStopRollerAction(intake, forceNew));
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
