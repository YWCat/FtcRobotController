package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RotatingSlide {
    public Arm arm = null;
    public DualMotorSlide slide = null;

    //Preset slide lengths in inches, need to be fine-tuned.
    public static int SLIDE_CHAMBER_PREP = 1000;
    public static int SLIDE_CHAMBER_PLACE = 6000;
    public static int SLIDE_BASKET = 1000;
    public static int SLIDE_HANG_PREP = 1000;
    public static int SLIDE_INTAKE = 1000;
    public static int SLIDE_STARTING = 0;


    public static int ARM_CHAMBER_PREP = 1000; //in ticks
    public static int ARM_CHAMBER_PLACE = 600; //in ticks
    public static int ARM_BASKET = 1000; //ticks
    public static int ARM_HANG_PREP = 800; //TODO: find actual pos w/ robot
    public static int ARM_INTAKE = 0; //in ticks
    public static int ARM_STARTING = 0;
    public static int ARM_VERTICAL_POS = 900;

    public RotatingSlide(){
        arm = new Arm();
        slide = new DualMotorSlide();
    }

    public Action rotSlideToPosition(int slidePos, int armPos) {
        int slideTargetPosition = slidePos;
        int armTargetPosition = armPos;

        int slideStartPosition = slide.getMotorLPosition();
        int armStartPosition = arm.getMotorPosition();

        /*
        If moving up and retracting, rotate arm and retract simultaneously
        If moving up and extending, rotate arm, then extend (no need to retract)
        If moving down regardless, retract, rotate, then adjust length
        */
        //We assume the safest option as the default
        Action action = new SequentialAction(
                slide.getSlideToPosition(SLIDE_STARTING, true),
                arm.getArmToPosition(armTargetPosition, false),
                slide.getSlideToPosition(slideTargetPosition, true)
            );

        /*
        //TODO: make the slightly more optimized version bug-friendly
        //There is no timeout within the entire action, so if one action fails

        //If the arm goes up (closer to vertical), change the mode
        if(Math.abs(armStartPosition - ARM_VERTICAL_POS) >  Math.abs(armTargetPosition - ARM_VERTICAL_POS)){
            action = new SequentialAction(
                    arm.getArmToPosition(armTargetPosition, false),
                    slide.getSlideToPosition(slideTargetPosition, false));
            //Determine if the slide is extending or retracting
            if (slideStartPosition < slideTargetPosition) {
                action = new ParallelAction(
                        arm.getArmToPosition(armTargetPosition, false),
                        slide.getSlideToPosition(slideTargetPosition, false));
            }
        }*/

        return action;
    }

}
