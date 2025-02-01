package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;
@Config
public class SampleIntakeClaw {
    Servo  intakeServo;
    DistanceSensor distSensor;
    Servo wristServo;
    public static moveClaw moveClawAction = null;
    public static waitAct waitAction = null;
    public static turnWrist turnWristAction = null;
    public static  double CLOSED_POS_CLAW = 0;
    public static  double OPEN_POS_CLAW = 1;

    public static  double distanceThreshold = 2; //in inches
    public static  double WRIST_INTAKE_CLAW = 0.455; //0.455;
    public static  double WRIST_ASCEND = 0.46;//0.49
    public static  double WRIST_PREP_OUTTAKE_CLAW = 0.365; //0.39;
    public static double WRIST_HANG_CLAW = 0.34;
    public static  double WRIST_OUTTAKE_CLAW = 0.325; //0.365;
    public static  double WRIST_INTAKE_SPECIMEN_CLAW = 0.4;
    public static  double WRIST_OUTTAKE_SPECIMEN_CLAW = 0.48;
    public static  double WRIST_IDLE_CLAW = 0.44;
    public static  double WRIST_INIT_CLAW = 0.48;

    public SampleIntakeClaw(){
        RobotCore robotCore = RobotCore.getRobotCore();
        intakeServo = robotCore.hardwareMap.get(Servo.class, RobotConfig.sampleServo);
        distSensor = robotCore.hardwareMap.get(DistanceSensor.class, RobotConfig.distanceSensor);
        wristServo = robotCore.hardwareMap.get(Servo.class, RobotConfig.wrist); //0.5 is intake, 0.9 is outtake, base pos = 1.0, hold pos = 0.8
        //Log.v("intakeServo initialized", "power = " + 0);
        wristServo.setPosition(WRIST_INIT_CLAW);

    }
    public double getWristPosition(){
        return wristServo.getPosition();
    }

    public final class moveClaw implements Action {
        private boolean cancelled = false;
        private long startTime;
        private double servoPos; //
        private boolean runStarted;
        private DcMotorSimple.Direction direction;
        public moveClaw(boolean isClose){
            changeTarget(isClose);
            //Log.i(" intakeServoStartRoller RobotActions", "Created new action startAndStopRoller");
        }
        public void changeTarget(boolean isClose){
            if(isClose){
                servoPos = CLOSED_POS_CLAW;
            } else{
                servoPos = OPEN_POS_CLAW;
            }
            Log.i("intakeServo moveClaw", "Created new action: closing claw = " + isClose);
            runStarted = false;
            cancelled = false;

        }

        public boolean run (@NonNull TelemetryPacket p){
            intakeServo.setPosition(servoPos);
            Log.i("intakeServo moveClaw RobotActions", "set position: " + servoPos);
            startTime = System.currentTimeMillis();
            runStarted = true;
            return false;
        }
        public void cancel(){ //call this AFTER clearing the actions list in LoopUpdater
            cancelled = true;
            Log.i("intakeServo moveClaw RobotActions", "moveClaw action cancelled");
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
            Log.i(" intakeServo wrist RobotActions", "Created new action turnWrist to position " + position);
        }
        public void changeTarget(double position){
            wristPos = position;
            Log.i(" intakeServo wrist RobotActions", "action modified to position " + position);
            runStarted = false;
            cancelled = false;

        }

        public boolean run (@NonNull TelemetryPacket p){
            if(!runStarted){
                Log.i(" intakeServo wrist RobotActions", "going to position " + wristPos);
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
            cancelled = true;
            //Log.i("intakeServo RobotActions", "action cancelled");
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
            cancelled = true;
            //Log.i("intakeServoStartRoller RobotActions", "action cancelled");
        }
    }


    public moveClaw getMoveClawAction(boolean isClose, boolean forceNew){
        if(!forceNew){
            if(moveClawAction == null){
                moveClawAction = new moveClaw(isClose);
            } else {
                moveClawAction.changeTarget(isClose);
            }
            return moveClawAction;
        } else{
            return new moveClaw(isClose);
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



}
