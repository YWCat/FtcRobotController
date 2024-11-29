package org.firstinspires.ftc.teamcode.utility;


import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.SampleIntake;


public class SmartGamepad extends Gamepad implements PeriodicUpdatingEntity{
    private LoopUpdater loopUpdater = null;
    private waitForButton waitForButtonAction = null;

    private Gamepad original = null;  // reference to the original gamepad object from opmode
    private Gamepad previous = null;  // a copy of gamepad with previous state

    public SmartGamepad(Gamepad gamepad) {
        original = gamepad;
        previous = new Gamepad();
        loopUpdater = LoopUpdater.getSharedLoopUpdater();
        loopUpdater.addPeriodicUpdatingEntity(this);
        //Log.v("gamepad", "Smart gamepad check in.");
    }

    public void update (TelemetryPacket packet) {
        previous.copy(this);
        copy(original);
    }

    public boolean dpad_up_pressed() {
        return dpad_up && !previous.dpad_up;
    }

    public boolean dpad_down_pressed() {
        return dpad_down && !previous.dpad_down;
    }

    public boolean dpad_left_pressed() {
        return dpad_left && !previous.dpad_left;
    }

    public boolean dpad_right_pressed() {
        return dpad_right && !previous.dpad_right;
    }

    public boolean left_trigger_pressed() {
        return left_trigger>0.1 && !(previous.left_trigger>0.1);
    }
    public boolean right_trigger_pressed() {
        return right_trigger>0.1 && !(previous.right_trigger>0.1);
    }

    public boolean a_pressed() {
        return a && !previous.a;
    }

    public boolean b_pressed() {
        return b && !previous.b;
    }

    public boolean x_pressed() {
        return x && !previous.x;
    }

    public boolean y_pressed() {
        return y && !previous.y;
    }

    public boolean left_bumper_pressed(){
        return left_bumper && !previous.left_bumper;
    }
    public boolean right_bumper_pressed(){
        return right_bumper && !previous.right_bumper;
    }
    public boolean dpad_changed() {
        //Log.v("gamepad", String.format("Dpad (up/down/left/right): %b,%b,%b,%b, previous: %b,%b,%b,%b", dpad_up, dpad_down, dpad_left, dpad_right,
        //        previous.dpad_up, previous.dpad_down, previous.dpad_left, previous.dpad_right));

        boolean result =  (dpad_up ^ previous.dpad_up)     || (dpad_down ^ previous.dpad_down) ||
                (dpad_left ^ previous.dpad_left) || (dpad_right ^ previous.dpad_right);
        if (result) {
            //Log.v("gamepad/transition", "dpad changed: True");
        }
        return result;
    }

    public boolean right_stick_released() {
        boolean result =  (Math.abs(right_stick_x) + Math.abs(right_stick_y) < 0.001) &&
                (Math.abs(previous.right_stick_x) + Math.abs(previous.right_stick_y) > 0.001);
        if (result) {
            //Log.v("gamepad/transition", "right stick released: True");
        }
        return result;
    }
    public boolean left_trigger_released(){
        return !(left_trigger>0.5) && previous.left_trigger>0.5;
    }
    public boolean right_trigger_released(){
        return !(right_trigger>0.5) && previous.right_trigger>0.5;
    }

    public boolean left_stick_button_pressed() {
        return left_stick_button && !previous.left_stick_button;
    }
    public boolean right_stick_button_pressed(){
        return right_stick_button && !previous.right_stick_button;
    }

    public class waitForButton implements Action {
        String button = "";
        boolean cancelled = false;
        long timeout = 10000;

        public waitForButton(String button){
            changeSettings(button);
            Log.i(" smartGamepad RobotActions", "Created new action waitForButton");
        }
        public void changeSettings(String button){
            this.button = button;;
            cancelled = false;
        }

        public boolean run(@NonNull TelemetryPacket p){
            if(!cancelled){
                if(button.equals("left_stick_y_forward")){
                    return !(left_stick_y > 0.3);
                } else if (button.equals("left_trigger")){
                    if(left_trigger_pressed()){
                        Log.v("RobotActions WaitForButton", "left trig pressed");
                    }
                    return !left_trigger_pressed();
                } else if (button.equals("right_trigger")){
                    if(right_trigger_pressed()){
                        Log.v("RobotActions WaitForButton", "right trig pressed");
                    }
                    return !right_trigger_pressed();
                } else if(button.equals("a")){
                    if(a_pressed()){
                        Log.v("RobotActions WaitForButton", "a pressed");
                    }
                    return !a_pressed();
                }

                else{
                    Log.e("RobotActions WaitForButton", "BUTTON NOT SUPPORTED");
                    return false;
                }
            } else {
                return false;
            }

        }
        public void cancel(){
            cancelled = true;
            Log.i("intakeServo RobotActions", "action cancelled");
        }
    }

    public waitForButton getWaitForButton(String button, boolean forceNew){
        if(!forceNew){
            if(waitForButtonAction == null){
                waitForButtonAction = new waitForButton(button);
            } else {
                waitForButtonAction.changeSettings(button);
            }
            return waitForButtonAction;
        } else{
            return new waitForButton(button);
        }
    }

}
