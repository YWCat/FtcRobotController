package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.RobotApplication;
import org.firstinspires.ftc.teamcode.subsystems.DualMotorSlide;
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SampleIntake;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenIntake;
import org.firstinspires.ftc.teamcode.utility.LoopUpdater;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;
import org.firstinspires.ftc.teamcode.utility.SmartGamepad;

@TeleOp
@Config
public class AATele extends LinearOpMode{
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    //TODO: replace this with smartGamepad
    private SmartGamepad smartGamepad1 = null;
    private SmartGamepad smartGamepad2 = null;
    private static final double MIN_SLOW_MODE = 0.4;
    private static final double DRIVE_SLOW_MODE = 0.3;
    private boolean slowModeOn = true;
    private  boolean specimenOpenToggle = false;
    private int intakeMode = 0;
    private int outtakeMode = 0;


    @Override
    public void runOpMode() {
        RobotCore robotCore  = new RobotCore(this);
        LoopUpdater loopUpdater = new LoopUpdater(); //I'm not sure if I'm doing it right
        RotatingSlide rotatingSlide = new RotatingSlide();
        SpecimenIntake specimenIntake = new SpecimenIntake(); //actually the most useless class, but its for the sake of abstraction
        SampleIntake sampleIntake = new SampleIntake();

        smartGamepad1 = new SmartGamepad(gamepad1);
        smartGamepad2 = new SmartGamepad(gamepad2);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, RobotConfig.leftFront);
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, RobotConfig.leftRear);
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, RobotConfig.rightFront);
        rightBackDrive = hardwareMap.get(DcMotorEx.class, RobotConfig.rightRear);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;
            TelemetryPacket packet = new TelemetryPacket();

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double leftJoystickRadius = Math.min(1,Math.sqrt(Math.pow(gamepad1.left_stick_y,2) + Math.pow(gamepad1.left_stick_x,2)));
            double rightJoystickRadius = Math.abs(gamepad1.right_stick_x);
            lateral = robotCore.mapJsComponents(gamepad1.left_stick_x, leftJoystickRadius, slowModeOn);
            axial = robotCore.mapJsComponents(-gamepad1.left_stick_y, leftJoystickRadius, slowModeOn);
            yaw = robotCore.mapJsComponents(gamepad1.right_stick_x, rightJoystickRadius, slowModeOn);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));


            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            if(smartGamepad1.left_stick_button || smartGamepad1.right_stick_button){
                slowModeOn = true;
            } else{
                slowModeOn = false;
            }
            /*
            if (slowModeOn){
                leftFrontPower  *= DRIVE_SLOW_MODE;
                rightFrontPower *= DRIVE_SLOW_MODE;
                leftBackPower   *= DRIVE_SLOW_MODE;
                rightBackPower  *= DRIVE_SLOW_MODE                     ;
            }*/

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);



            if(smartGamepad2.x_pressed()){ //retract slides and make arm vertical
                loopUpdater.addAction(rotatingSlide.retractSlide());
                //Log.i("UpdateActions", "Added retract action; Actions Active: " +  loopUpdater.getActiveActions().size());
            } else if (smartGamepad2.b_pressed()){ // go to chamber
                /*Action armToChamber = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_CHAMBER_PREP_TICKS,  false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN, 1, true);
                Action toChamber = new SequentialAction(retractSlide, armToChamber, extendSlide);
                loopUpdater.addAction(toChamber);
                 */
                //Log.i("UpdateActions", "Actions Active: " +  loopUpdater.getActiveActions().size());
            }

            if (smartGamepad2.a_pressed()){ // put everything in position to intake
                Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AFTER_INTAKE, false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 0.5, false);
                Action prepIntake = sampleIntake.getPrepIOAction(true, false);
                Action rollerOn = sampleIntake.getStartRollerAction(true, false);
                Action waitforB = smartGamepad2.getWaitForButtons("b", false);
                Action pickSample = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS, true);

                Action toIntake = new ParallelAction(
                        rollerOn,
                        new SequentialAction(retractSlide, armToIntake, prepIntake, waitforB, pickSample));
                if(rotatingSlide.isHorizontal()){
                    toIntake = new ParallelAction(
                            rollerOn,
                            new SequentialAction(armToIntake, prepIntake, waitforB, pickSample));
                }
                //loopUpdater.addAction(toIntake);

                //pick up sample stuff
                Action raiseArmAfterIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AFTER_INTAKE, true);
                Action waitForB1 = smartGamepad2.getWaitForButtons("b", true);
                Action waitForB2 = smartGamepad2.getWaitForButtons("b", true);
                Action retractSlideAfterIntake = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 0.5, true);
                Action retractArmAfterIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_RETRACT, true);
                Action intakeSequence = new SequentialAction(waitForB1, raiseArmAfterIntake, waitForB2, retractSlideAfterIntake, retractArmAfterIntake);
                loopUpdater.addAction(new SequentialAction(toIntake, intakeSequence));
                //Log.i("UpdateActions", "Intake prep added;  Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad2.y_pressed()){ //basket outtake prep

                Action armToBasket = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_BASKET_TICKS,false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN, 1, true);
                Action closeClaw = sampleIntake.getStartAndStopRollerAction(true, false);
                Action turnWrist = sampleIntake.getTurnWristAction(SampleIntake.WRIST_OUTTAKE_ROLLER, false);
                Action toOuttake = new ParallelAction(new SequentialAction( retractSlide, armToBasket , extendSlide, turnWrist));
                loopUpdater.addAction(toOuttake);
                //Log.i("UpdateActions", "Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad1.a_pressed()){
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, 1, false);
                Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_TICKS, false);
                Action prepIntake = sampleIntake.getPrepIOAction(true, false);
                Action rollerOn = sampleIntake.getStartAndStopRollerAction(false, false);
                Action rollerOff = sampleIntake.getStopRollerAction(false);
                Action raiseArm1 = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AFTER_INTAKE, true);
                Action raiseArm2 = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_RETRACT, true);
                Action toOuttakeSample = new SequentialAction(retractSlide,
                        armToIntake, prepIntake, rollerOn,
                        raiseArm2, rollerOff);
                loopUpdater.addAction(toOuttakeSample);
                //Log.i("UpdateActions", "Intake prep added;  Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad1.b_pressed()){
                rotatingSlide.updateHangStatus(true);
                Action flipWristAway = sampleIntake.getTurnWristAction(SampleIntake.WRIST_HANG_ROLLER, false);
                Action lowerArmLow = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_LOW_TICKS, true);
                Action lowerSlideLowFirst = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_LOW_FIRST_IN, 0.5, true);
                Action lowerSlideLowSecond = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_LOW_IN, 0.8, true);
                Action lowerArmLowLock = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_LOW_LOCK_TICKS, true);
                Action lowerSlideLowLock = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_LOW_LOCK_IN, 0.8, true);
                Action waitForLB1 = smartGamepad2.getWaitForButtons("left_bumper", false);

                Action powerMotorsHold = rotatingSlide.slide.getSetMotorPower(DualMotorSlide.HOLD_POWER_HANG, false);

                Action raiseSlidePrep= rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_HIGH_PREP_IN, 0.8, true);
                Action raiseArmPrep = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_HIGH_PREP_TICKS, true);
                Action waitForLB2 = smartGamepad2.getWaitForButtons("left_bumper", true);
                Action raiseArmSwing = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_HIGH_SWING_TICKS, true);
                Action lowerSlideSwing = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_HIGH_SWING_IN, 0.8, true);
                Action powerMotorsHoldLock = rotatingSlide.slide.getSetMotorPower(DualMotorSlide.HOLD_POWER_HANG, true);
                Action lowerArmLock = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_HIGH_LOCK_TICKS, true);
                Action lowerSlideLock = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_HIGH_LOCK_IN, 0.8, true);
                Action hangSequence = new SequentialAction(
                        new ParallelAction(flipWristAway, lowerArmLow, new SequentialAction(lowerSlideLowFirst, lowerSlideLowSecond)), lowerArmLowLock, powerMotorsHold,
                        waitForLB1,
                        raiseSlidePrep, raiseArmPrep,
                        waitForLB2,
                        new ParallelAction(raiseArmSwing, lowerSlideSwing), powerMotorsHoldLock, new SleepAction(1),  lowerArmLock, lowerSlideLock
                        );
                loopUpdater.addAction(hangSequence);
            }
            /*
            if(smartGamepad1.right_trigger_pressed()){ //start rollers INTAKE or pick up and go
                Action rollIntake = sampleIntake.getStartAndStopRollerAction(true, false);
                Action turnWristDown = sampleIntake.getTurnWristAction(SampleIntake.WRIST_IDLE_ROLLER, false);
                Action outtakeAction = new SequentialAction(rollIntake, turnWristDown);
                Log.i("UpdateActions", "start intake; Actions Active: " +  loopUpdater.getActiveActions().size());
                loopUpdater.addAction(outtakeAction);
            }*/
            /*
            switch (outtakeMode){
                case 1:
                    sampleIntake.manualMoveRoller(SampleIntake.OUTTAKE_POWER_ROLLER);

            }
            */
            if (smartGamepad1.left_trigger_pressed() && !smartGamepad1.dpad_up){ //outtake + drop and go down
                Action rollOuttake = sampleIntake.getStartAndStopRollerAction(false, false);
                Action turnWristDown = sampleIntake.getTurnWristAction(SampleIntake.WRIST_IDLE_ROLLER, false);
                Action waitForLT = smartGamepad1.getWaitForButtons("left_trigger", false);
                Action outtakeAndLeave = new SequentialAction(rollOuttake,
                        new ParallelAction(turnWristDown, new SleepAction(0)),
                        rotatingSlide.retractSlide());
                loopUpdater.addAction(outtakeAndLeave);
            }

            if(gamepad1.dpad_up &&  gamepad1.left_trigger>0.5){
                sampleIntake.manualMoveRoller(SampleIntake.OUTTAKE_POWER_ROLLER);
            } else if (gamepad1.dpad_up && smartGamepad1.left_trigger_released()){
                sampleIntake.manualMoveRoller(0);
            }
            if(gamepad1.dpad_up && gamepad1.right_trigger>0.5){
                sampleIntake.manualMoveRoller(SampleIntake.INTAKE_POWER_ROLLER);
            } else if (gamepad1.dpad_up && smartGamepad1.right_trigger_released()){
                sampleIntake.manualMoveRoller(0);
            }

            /*
            switch (intakeMode) {
                case 1:
                    if(smartGamepad1.right_trigger_pressed()){
                        Action raiseArm = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_AFTER_INTAKE, false);
                        loopUpdater.addAction(raiseArm);
                        intakeMode = 2;
                    }
                    Log.v("intakemode", ""+intakeMode);
                    break;
                case 2:
                    if(smartGamepad1.right_trigger_pressed()){
                        loopUpdater.addAction(rotatingSlide.retractSlide());
                        intakeMode = 0;
                    }
                    Log.v("intakemode", ""+ x);
                    break;
                default:
                    if(smartGamepad1.right_trigger_pressed()){
                        sampleIntake.manualMoveRoller(SampleIntake.INTAKE_POWER_ROLLER);
                        intakeMode = 1;
                    }
                    Log.v("intakemode", ""+intakeMode);
                    break;

            }

             */
            if(smartGamepad1.right_bumper_pressed()){ //intake specimen and move slightly up
                Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.CLOSE, false);
                if(!smartGamepad1.dpad_up){ //manual mode turns this off
                    Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN, 0.4, false);
                    loopUpdater.addAction(new SequentialAction(closeSpecimen, raiseSlide));
                } else {
                    loopUpdater.addAction(closeSpecimen);
                }

            } if (smartGamepad1.left_bumper_pressed()){
                Action openSpecimen= specimenIntake.getMoveSpecimenIntake(SpecimenIntake.OPEN, false);
                if(!smartGamepad1.dpad_up){ //manual mode turns this off
                    Action lowerSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 0.4,  false);
                    loopUpdater.addAction(new SequentialAction(lowerSlide, openSpecimen));
                } else{
                    loopUpdater.addAction(openSpecimen);
                }
            }


            if(gamepad2.dpad_up){
                rotatingSlide.slide.adjustLift(1 * Math.min(1, (gamepad2.right_trigger+MIN_SLOW_MODE)));
            }  if(gamepad2.dpad_down){
                rotatingSlide.slide.adjustLift(-1 * Math.min(1, (gamepad2.right_trigger+MIN_SLOW_MODE)));
            }  if(gamepad2.dpad_right){
                Action moveArm = rotatingSlide.arm.getArmToPosition(rotatingSlide.arm.getMotorPosition() + (int) (250 * Math.min(1, (gamepad2.right_trigger+MIN_SLOW_MODE))), false);
                loopUpdater.addAction(moveArm);
            }  if(gamepad2.dpad_left){
                Action moveArm = rotatingSlide.arm.getArmToPosition(rotatingSlide.arm.getMotorPosition() - (int) (250 * Math.min(1, (gamepad2.right_trigger+MIN_SLOW_MODE))), false);
                loopUpdater.addAction(moveArm);
            } if ((!gamepad2.dpad_down && !gamepad2.dpad_up) && rotatingSlide.slide.isLevelReached()){
                //Log.v("manualControl AATele slide holdPosition", "is target reached? " + rotatingSlide.slide.isLevelReached());
                Log.v("Slide Power", "invoking holdPosition at 3");
                rotatingSlide.slide.holdPosition();
            }

            if(smartGamepad2.left_bumper_pressed()){
                double pos = sampleIntake.getWristPosition();
                Action wristIntake = sampleIntake.getTurnWristAction(pos + 0.05 * Math.min(1, (1-gamepad2.right_trigger+MIN_SLOW_MODE)), false);
                loopUpdater.addAction(wristIntake);
            }
            if(smartGamepad2.right_bumper_pressed()){
                double pos = sampleIntake.getWristPosition();
                Action wristIntake = sampleIntake.getTurnWristAction(pos - 0.05 * Math.min(1, (1-gamepad2.right_trigger+MIN_SLOW_MODE)), false);
                loopUpdater.addAction(wristIntake);
            }




            if(smartGamepad2.left_stick_button_pressed() || smartGamepad1.dpad_down_pressed()){
                loopUpdater.clearActions();
                rotatingSlide.updateHangStatus(false);
                intakeMode = 0;
                //sampleIntake.manualMoveRoller(0);
                rotatingSlide.slide.holdPosition();
                rotatingSlide.arm.stopMotor();
            }
            if(smartGamepad2.right_stick_button_pressed()){
                rotatingSlide.slide.resetEncoder();
                rotatingSlide.arm.resetEncoder();
            }

            //experimental features

            if(smartGamepad2.left_trigger_pressed()){
                rotatingSlide.updateHangStatus(true);
                Action lowerSlideLowFirst = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_LOW_FIRST_IN, 0.5, true);
                Action lowerSlideLowSecond = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_LOW_IN, 0.8, true);
                loopUpdater.addAction(new SequentialAction(
                        lowerSlideLowFirst,
                        new SleepAction(2),
                        lowerSlideLowSecond
                ));
            }


            rotatingSlide.update();
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Slides Position", "left: "  + rotatingSlide.slide.getLeftEncoder() + "right: " + rotatingSlide.slide.getRightEncoder());
            telemetry.addData("Slides Position Inches", "left: "  + rotatingSlide.slide.getPosition());
            telemetry.addData("slides velocity", "left: " + rotatingSlide.slide.getLeftVelocity() + "right: " + rotatingSlide.slide.getRightVelocity());
            telemetry.addData("worm position", ""+ rotatingSlide.arm.getMotorPosition());
            telemetry.addData("wrist position", ""+ sampleIntake.getWristPosition());
            telemetry.addData("# Active Actions: ", loopUpdater.getActiveActions().size());
            telemetry.update();
            loopUpdater.updateAndRunAll();


        }
    }
}