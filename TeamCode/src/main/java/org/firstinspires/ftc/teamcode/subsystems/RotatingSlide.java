package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

@Config
public class RotatingSlide {
    public Arm arm = null;
    public DualMotorSlide slide = null;

    public boolean IS_HANGING = false;

    //Preset slide lengths in inches, need to be fine-tuned.
    //CHAMBER
    public static final double SLIDE_CHAMBER_PREP_IN = 17.5;
    public static final int ARM_CHAMBER_PREP_TICKS = 0; //in ticks
    public static final double SLIDE_CHAMBER_PLACE_IN = 13;
    public static final double SLIDE_PICK_UP_SPECIMEN_IN = 2.9;
    public static final int ARM_CHAMBER_PLACE_TICKS = 0; //in ticks

    //BASKET
    public static final double SLIDE_BASKET_IN = 29.5;
    public static  int ARM_BASKET_TICKS = -280; //ticks
    public static double ARM_BASKET_DEG = -9.363; //ticks
    public static int ARM_AUTO_BASKET_TICKS = 380; // roller intake
    public static int ARM_AUTO_BASKET_TICKS_CLAW = (-400+100);
    public static double ARM_AUTO_ASCEND_DEG = 25;
    public static double ARM_AUTO_BASKET_DEG = 12.707;

    //HANG
    public static  int SLIDE_HANG_PREP_TICKS = 1800;
    public static  int ARM_HANG_PREP_TICKS = 820;
    public static double ARM_HANG_PREP_DEG = 27.420;
    public static int ARM_HANG_LOW_TICKS = 2600;
    public static double ARM_HANG_LOW_DEG = 86.940;
    public static double SLIDE_HANG_LOW_FIRST_IN = 7;
    public static double SLIDE_HANG_LOW_IN = 0.8;

    public static int ARM_HANG_LOW_LOCK_TICKS = 2020;
    public static double ARM_HANG_LOW_LOCK_DEG = 67.545;
    public static double SLIDE_HANG_LOW_LOCK_IN = 0.0;
    public static double SLIDE_HANG_HIGH_PREP_IN = 20.0;
    public static double ARM_HANG_HIGH_PREP_PREP_DEG = 75;
    public static double ARM_HANG_HIGH_PREP_DEG = 83;
    public static double ARM_HANG_HIGH_SWING_DEG = 8.032;
    public static double SLIDE_HANG_HIGH_SWING_IN = 4;
    public static double ARM_HANG_HIGH_LOCK_DEG = -8.025;
    public static double SLIDE_HANG_HIGH_LOCK_IN = -0.3;



    //INTAKE + IDLE
    public static  int SLIDE_INTAKE_TICKS = 1300;
    public static  int ARM_INTAKE_TICKS = 2400; //in ticks
    public static double ARM_INTAKE_DEG = 90.0;
    public static  double ARM_ABOVE_INTAKE_LOW_DEG = 78.75;
    public static  double ARM_ABOVE_INTAKE_LOWEST_DEG = 82.5;
    public static  double ARM_ABOVE_INTAKE_HIGH_DEG = 67.5;
    public static  double SLIDE_RETRACT_IN = 0;
    public static  int ARM_VERTICAL_POS = 0;

    public static  int ARM_RETRACT = ARM_VERTICAL_POS;
    public static double ARM_HORIZONTAL_THRESHOLD_DEG = 45.0;




    public RotatingSlide(){
        arm = new Arm(this);
        slide = new DualMotorSlide(this);
        IS_HANGING = false;
    }

    /*
    used after the intake is done
    sequential actions: wait for intake roller to finish, raise slide slightly, retract slides all the way
     */
    public Action prepIntake(){
        Action armToIntake = arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS, false);
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, false);
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
                sampleIntake.getStartAndStopRollerAction(SampleIntakeRoller.ROLLER_POWER*-1, false)
        );
    }*/

    /*
    Retracts the slide to 0 and makes the arm vertical
     */
    public Action retractSlide(){
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, true);
        Action retractArm = arm.getArmToPosition(RotatingSlide.ARM_RETRACT, true);
        Action retract = new ParallelAction(retractSlide, retractArm);
        Log.i("UpdateActions", "Added retract action");
        return retract;
    }
    //lets you customize if you want one thing to not move
    public Action retractSlide(boolean moveSlide, boolean moveArm){
        Action retractSlide = slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, false);
        Action retractArm = arm.getArmToPosition(RotatingSlide.ARM_RETRACT,false);
        if(!moveSlide){
            retractSlide = slide.getSlideToPosition(slide.getRightEncoder(),1,  false);
        }
        if(!moveArm){
            retractArm = arm.getArmToPosition(arm.getMotorPositionTicks(),false);
        }
        Action retract = new SequentialAction(retractSlide, retractArm);
        Log.i("UpdateActions", "Added retract action, moveSlide:" + moveSlide + " moveArm: " + moveArm);
        return retract;
    }

    public void update(){
        slide.changeHorizontalSetting();
    }
    public double getArmEffectiveAngle(){
        double angle = arm.getMotorPositionAngle();
        if(IS_HANGING){
            angle = 0;
        }
        return angle;
    }

    public double getHorizontalExpansionLength(){
        double effLength = slide.getPosition() * Math.sin(getArmEffectiveAngle() * Math.PI / 180);
        //Log.i("horizontal expansion length", ""+ effLength);
        return effLength;
    }
    public double getSlideMaxHorizontalLimitInches(){
        return slide.getMaxHorizontalLimitInches();
    }
    public boolean getSlideExceedsHorizontalLimit(){
        return slide.getExceedsHorizontalLimit(0);
    }
    public boolean getSlideExceedsHorizontalLimit(double clearance){
        return slide.getExceedsHorizontalLimit(clearance);
    }
    public double getHorizontalThresholdAngle(){
        return ARM_HORIZONTAL_THRESHOLD_DEG;
    }

    public boolean isHorizontal(){
        boolean horizontal = getArmEffectiveAngle() > ARM_HORIZONTAL_THRESHOLD_DEG;
        //boolean horizontal = arm.getMotorPositionAngle() > ARM_HORIZONTAL_THRESHOLD_DEG && !IS_HANGING;
        Log.i("horizontalLimit 3", "isHorizontal function, " + horizontal);
        if (horizontal) {
            return true;
        } else{
            return false;
        }
    }

    public void updateHangStatus(boolean isHanging){
        IS_HANGING = isHanging;
        Log.v("horizontalLimit2", "Update status " + isHanging);
    }

    public boolean getIsHanging(){
        return IS_HANGING;
    }


}
