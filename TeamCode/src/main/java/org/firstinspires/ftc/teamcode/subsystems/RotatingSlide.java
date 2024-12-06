package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

@Config
public class RotatingSlide {
    public Arm arm = null;
    public DualMotorSlide slide = null;

    public static boolean IS_HANGING = false;

    //Preset slide lengths in inches, need to be fine-tuned.
    //CHAMBER
    public static final double SLIDE_CHAMBER_PREP_IN = 15.63;
    public static final int ARM_CHAMBER_PREP_TICKS = 0; //in ticks
    public static final double SLIDE_CHAMBER_PLACE_IN = 11.59;
    public static final double SLIDE_CHAMBER_HI_IN = 16;
    public static final int SLIDE_PICK_UP_SPECIMEN_TICKS = 800;
    public static final double SLIDE_PICK_UP_SPECIMEN_IN = 6.10;
    public static final int ARM_CHAMBER_PLACE_TICKS = 0; //in ticks

    //BASKET
    public static final double SLIDE_BASKET_IN = 29.5;
    public static  int ARM_BASKET_TICKS = -280; //ticks
    public static int ARM_AUTO_BASKET_TICKS = 380;

    //HANG
    public static  int SLIDE_HANG_PREP_TICKS = 1800;
    public static  int ARM_HANG_PREP_TICKS = 820;
    public static int ARM_HANG_LOW_TICKS = 2600;
    public static double SLIDE_HANG_LOW_IN = 0.8;

    public static int ARM_HANG_LOW_LOCK_TICKS = 2020;
    public static double SLIDE_HANG_LOW_LOCK_IN = 0.0;
    public static double SLIDE_HANG_HIGH_PREP_IN = 20.0;
    public static int ARM_HANG_HIGH_PREP_TICKS = 2270;
    public static int ARM_HANG_HIGH_SWING_TICKS = 300;
    public static double SLIDE_HANG_HIGH_SWING_IN = 1.5;
    public static int ARM_HANG_HIGH_LOCK_TICKS = -240;
    public static double SLIDE_HANG_HIGH_LOCK_IN = -0.3;



    //INTAKE + IDLE
    public static  int SLIDE_INTAKE_TICKS = 1300;
    public static  int ARM_INTAKE_TICKS = 2400; //in ticks
    public static  int ARM_AFTER_INTAKE = 2200;
    public static  int SLIDE_RETRACT = 0;
    public static  int ARM_VERTICAL_POS = 0;
    public static  int ARM_RETRACT = ARM_VERTICAL_POS;

    public static int ARM_HORIZONTAL_THRESHOLD = 1300;




    public RotatingSlide(){
        arm = new Arm();
        slide = new DualMotorSlide();
        IS_HANGING = false;
    }

    /*
    used after the intake is done
    sequential actions: wait for intake roller to finish, raise slide slightly, retract slides all the way
     */
    public Action prepIntake(){
        Action armToIntake = arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS, false);
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, false);
        Action extendSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_INTAKE_TICKS, 1, true);
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
                arm.getArmToPosition(ARM_BASKET_TICKS, false),
                slide.getSlideToPosition(SLIDE_BASKET_TICKS, false),
                sampleIntake.getStartAndStopRollerAction(SampleIntake.ROLLER_POWER*-1, false)
        );
    }*/

    /*
    Retracts the slide to 0 and makes the arm vertical
     */
    public Action retractSlide(){
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, false);
        Action retractArm = arm.getArmToPosition(RotatingSlide.ARM_RETRACT, false);
        Action retract = new SequentialAction(retractSlide, retractArm);
        Log.i("UpdateActions", "Added retract action");
        return retract;
    }
    //lets you customize if you want one thing to not move
    public Action retractSlide(boolean moveSlide, boolean moveArm){
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, false);
        Action retractArm = arm.getArmToPosition(RotatingSlide.ARM_RETRACT,false);
        if(!moveSlide){
            retractSlide = slide.getSlideToPosition(slide.getLeftEncoder(),1,  false);
        }
        if(!moveArm){
            retractArm = arm.getArmToPosition(arm.getMotorPosition(),false);
        }
        Action retract = new SequentialAction(retractSlide, retractArm);
        Log.i("UpdateActions", "Added retract action, moveSlide:" + moveSlide + " moveArm: " + moveArm);
        return retract;
    }

    public void update(){
        if (isHorizontal()) {
            slide.changeHorizontalSetting(true);
        } else{
            slide.changeHorizontalSetting(false);
        }
    }

    public boolean isHorizontal(){

        if (arm.getMotorPosition() > ARM_HORIZONTAL_THRESHOLD && !IS_HANGING) {
            return true;
        } else{
            return false;
        }
    }

    public void updateHangStatus(boolean isHanging){
        IS_HANGING = isHanging;
        Log.v("horizontalLimit2", "Update status " + isHanging);
    }


}
