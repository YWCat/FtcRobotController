package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

public class RotatingSlide {
    public Arm arm = null;
    public SampleIntake sampleIntake = null;
    public DualMotorSlide slide = null;

    //Preset slide lengths in inches, need to be fine-tuned.
    //CHAMBER
    public static int SLIDE_CHAMBER_PREP = 1000;
    public static int ARM_CHAMBER_PREP = 1000; //in ticks
    public static int SLIDE_CHAMBER_PLACE = 6000;
    public static int ARM_CHAMBER_PLACE = 600; //in ticks

    //BASKET
    public static int SLIDE_BASKET = 1000;
    public static int ARM_BASKET = 1000; //ticks

    //HANG
    public static int SLIDE_HANG_PREP = 1000;
    public static int ARM_HANG_PREP = 800; //TODO: find actual pos w/ robot

    //INTAKE + IDLE
    public static int SLIDE_INTAKE = 1000;
    public static int ARM_INTAKE = 0; //in ticks
    public static int ARM_AFTER_INTAKE = 100;
    public static int SLIDE_RETRACT = 0;
    public static int ARM_VERTICAL_POS = 900;
    public static int ARM_RETRACT = ARM_VERTICAL_POS;



    public RotatingSlide(){
        arm = new Arm();
        slide = new DualMotorSlide();
        sampleIntake = new SampleIntake();
    }

    /*
    used after the intake is done
    sequential actions: wait for intake roller to finish, raise slide slightly, retract slides all the way
     */
    public Action intakeSample(){
        return new SequentialAction(
                sampleIntake.getStartRollerAction(SampleIntake.ROLLER_POWER, false),
                arm.getArmToPosition(ARM_AFTER_INTAKE, false),
                slide.getSlideToPosition(SLIDE_RETRACT, false)
        );
    }

    /*
    used when player begins outtake sequence on basket
    sequential actions: raise slide to vertical, extend slide, flip outtake arm
     */
    public Action outtakeSample(){
        return new SequentialAction(
                arm.getArmToPosition(ARM_BASKET, false),
                slide.getSlideToPosition(SLIDE_BASKET, false),
                sampleIntake.getStartRollerAction(SampleIntake.ROLLER_POWER*-1, false)
        );
    }

    //Actually bad, keep the number of usages to zero please
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
                slide.getSlideToPosition(SLIDE_RETRACT, true),
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
