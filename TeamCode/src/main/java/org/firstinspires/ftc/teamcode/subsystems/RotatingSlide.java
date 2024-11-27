package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

public class RotatingSlide {
    public Arm arm = null;
    public DualMotorSlide slide = null;

    //Preset slide lengths in inches, need to be fine-tuned.
    //CHAMBER
    public static final int SLIDE_CHAMBER_PREP = 2160;
    public static final int ARM_CHAMBER_PREP = 0; //in ticks
    public static final int SLIDE_CHAMBER_PLACE = 1650;
    public static final int SLIDE_PICK_UP_SPECIMEN = 800;
    public static final int ARM_CHAMBER_PLACE = 0; //in ticks

    //BASKET
    public static final int SLIDE_BASKET = 3600;
    public static final int ARM_BASKET = -280; //ticks

    //HANG
    public static final int SLIDE_HANG_PREP = 1000;
    public static final int ARM_HANG_PREP = 800; //TODO: find actual pos w/ robot

    //INTAKE + IDLE
    public static final int SLIDE_INTAKE = 1300;
    public static final int ARM_INTAKE = 2722; //in ticks
    public static final int ARM_AFTER_INTAKE = 2675;
    public static final int SLIDE_RETRACT = 0;
    public static final int ARM_VERTICAL_POS = 0;
    public static final int ARM_RETRACT = ARM_VERTICAL_POS;




    public RotatingSlide(){
        arm = new Arm();
        slide = new DualMotorSlide();
    }

    /*
    used after the intake is done
    sequential actions: wait for intake roller to finish, raise slide slightly, retract slides all the way
     */
    public Action prepIntake(){
        Action armToIntake = arm.getArmToPosition(RotatingSlide.ARM_INTAKE, false);
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
        Action extendSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_INTAKE, true);
        Action toIntake = new SequentialAction(retractSlide, armToIntake, extendSlide/**/);
        return new SequentialAction(

        );
    }

    /*
    used when player begins outtake sequence on basket
    sequential actions: raise slide to vertical, extend slide, flip outtake arm
     */
    /*
    public Action outtakeSample(){
        return new SequentialAction(
                arm.getArmToPosition(ARM_BASKET, false),
                slide.getSlideToPosition(SLIDE_BASKET, false),
                sampleIntake.getStartRollerAction(SampleIntake.ROLLER_POWER*-1, false)
        );
    }*/

    /*
    Retracts the slide to 0 and makes the arm vertical
     */
    public Action retractSlide(){
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
        Action retractArm = arm.getArmToPosition(RotatingSlide.ARM_RETRACT,false);
        Action retract = new SequentialAction(retractSlide, retractArm);
        Log.i("UpdateActions", "Added retract action");
        return retract;
    }
    //lets you customize if you want one thing to not move
    public Action retractSlide(boolean moveSlide, boolean moveArm){
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
        Action retractArm = arm.getArmToPosition(RotatingSlide.ARM_RETRACT,false);
        if(!moveSlide){
            retractSlide = slide.getSlideToPosition(slide.getLeftEncoder(), false);
        }
        if(!moveArm){
            retractArm = arm.getArmToPosition(arm.getMotorPosition(),false);
        }
        Action retract = new SequentialAction(retractSlide, retractArm);
        Log.i("UpdateActions", "Added retract action, moveSlide:" + moveSlide + " moveArm: " + moveArm);
        return retract;
    }

    public void update(){
        if (arm.getMotorPosition() > 1000) {
            slide.changeHoldPower(true);
        } else{
            slide.changeHoldPower(false);
        }
    }

}
