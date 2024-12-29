package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
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
    private static final double MIN_SLOW_MODE = 0.3;
    private static final double DPAD_DRIVE = 0.3;
    private boolean slowModeOn = true;

    private Localizer localizer;
    private Pose2d pose;
    private final double robotLengthFromCOM = 14; //measured 12 inches long when arm is down, + 2 inch for safety
    private final Vector2d canLiftSlide = new Vector2d(-12, -24);
    private boolean trackingPosition = false;


    @Override
    public void runOpMode() {
        RobotCore robotCore  = new RobotCore(this);
        LoopUpdater loopUpdater = new LoopUpdater(); //I'm not sure if I'm doing it right
        RotatingSlide rotatingSlide = new RotatingSlide();
        SpecimenIntake specimenIntake = new SpecimenIntake(); //actually the most useless class, but its for the sake of abstraction
        SampleIntake sampleIntake = new SampleIntake();
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //the pose does not matter
        localizer = mecanumDrive.getDriveLocalizer();
        pose = new Pose2d(0,0,0);

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
        Action openSpecimenInit = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.OPEN, false);
        loopUpdater.addAction(openSpecimenInit);

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



            if(smartGamepad2.x_pressed()){ //retract slides and make arm vertical
                loopUpdater.addAction(rotatingSlide.retractSlide());
                //Log.i("UpdateActions", "Added retract action; Actions Active: " +  loopUpdater.getActiveActions().size());
            }

            if (smartGamepad2.a_pressed()){ // put everything in position to intake
                Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_ABOVE_INTAKE_DEG, false);
                Action retractSlide1 = rotatingSlide.slide.getSlideToPosition(Math.max(rotatingSlide.slide.getPosition()-1, RotatingSlide.SLIDE_RETRACT_IN), 1, false);
                Action retractSlide2 = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, true);
                Action prepIntake = sampleIntake.getPrepIOAction(true, false);
                Action rollerOn = sampleIntake.getStartRollerAction(true, false);
                Action waitForA = smartGamepad1.getWaitForButtons("a", false);
                Action pickSample = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE_DEG, true);

                Action toIntake = new ParallelAction(
                        rollerOn,
                        new SequentialAction(
                                retractSlide1,
                                new ParallelAction(retractSlide2, armToIntake),
                                prepIntake, waitForA, pickSample));
                if(rotatingSlide.isHorizontal()){
                    toIntake = new ParallelAction(
                            rollerOn,
                            new SequentialAction(armToIntake, prepIntake, waitForA, pickSample));
                }
                loopUpdater.addAction(toIntake);
                //Log.i("UpdateActions", "Intake prep added;  Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad1.x_pressed()){
                //after picking up a block for sample
                pose = new Pose2d(new Vector2d(0, 0), 0);
                Action raiseArmAfterIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_ABOVE_INTAKE_DEG, true);
                //Action triggerSamplePick = smartGamepad2.getWaitForButtons("x", true);
                loopUpdater.addAction(raiseArmAfterIntake);
                trackingPosition = true;

            } if(smartGamepad1.b_pressed()){
                Action raiseArmAfterIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_ABOVE_INTAKE_DEG, true);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, false);
                loopUpdater.addAction(new SequentialAction(raiseArmAfterIntake, retractSlide));
            }

            if(smartGamepad2.y_pressed() || /*basket outtake prep*/
                    (trackingPosition && (
                            pose.position.x + pose.heading.real*(rotatingSlide.getHorizontalExpansionLength()+robotLengthFromCOM) < 0
                    ))){
                trackingPosition = false;
                Action armToBasketPrep = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_VERTICAL_POS,false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET_IN, 1, true);
                Action turnWristPrep = sampleIntake.getTurnWristAction(SampleIntake.WRIST_PREP_OUTTAKE_ROLLER, false);
                Action armToBasket = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_BASKET_DEG, 0.5, true);
                Action turnWrist = sampleIntake.getTurnWristAction(SampleIntake.WRIST_OUTTAKE_ROLLER, true);
                Action toOuttake = new ParallelAction(
                        new SequentialAction(
                            new ParallelAction(turnWristPrep, armToBasketPrep, extendSlide),
                            armToBasket,
                            turnWrist));
                loopUpdater.addAction(toOuttake);
                //Log.i("UpdateActions", "Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad2.b_pressed()){ //outtake for specimen @ observation zone
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, false);
                Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_ABOVE_INTAKE_DEG, false);
                Action prepIntake = sampleIntake.getPrepIOAction(true, false);
                Action rollerOn = sampleIntake.getStartAndStopRollerAction(false, false);
                Action rollerOff = sampleIntake.getStopRollerAction(false);
                Action raiseArm1 = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_ABOVE_INTAKE_DEG, true);
                Action raiseArm2 = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_RETRACT, true);
                Action toOuttakeSample = new SequentialAction(
                        new ParallelAction(retractSlide, armToIntake),
                        prepIntake, rollerOn,
                        raiseArm2, rollerOff);
                loopUpdater.addAction(toOuttakeSample);
                //Log.i("UpdateActions", "Intake prep added;  Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad2.right_trigger >0.9 && smartGamepad2.left_bumper_pressed()){
                rotatingSlide.updateHangStatus(true);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Action flipWristAway = sampleIntake.getTurnWristAction(SampleIntake.WRIST_HANG_ROLLER, false);
                Action lowerArmLow = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_LOW_DEG, true);
                Action lowerSlideLowFirst = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_LOW_FIRST_IN, 0.5, true);
                Action lowerSlideLowSecond = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_LOW_IN, 0.8, true);
                Action lowerArmLowLock = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_LOW_LOCK_DEG, true);
                Action lowerSlideLowLock = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_LOW_LOCK_IN, 0.8, true);
                Action waitForLB1 = smartGamepad2.getWaitForButtons("left_bumper", false);

                Action powerMotorsHold = rotatingSlide.slide.getSetMotorPower(DualMotorSlide.HOLD_POWER_HANG, false);

                Action raiseSlidePrep= rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_HIGH_PREP_IN, 0.8, true);
                Action raiseArmPrep = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_HIGH_PREP_DEG, true);
                Action waitForLB2 = smartGamepad2.getWaitForButtons("left_bumper", true);
                Action raiseArmSwing = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_HIGH_SWING_DEG, true);
                Action lowerSlideSwing = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_HIGH_SWING_IN, 0.8, true);
                Action powerMotorsHoldLock = rotatingSlide.slide.getSetMotorPower(DualMotorSlide.HOLD_POWER_HANG, true);
                Action lowerArmLock = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_HANG_HIGH_LOCK_DEG, true);
                Action lowerSlideLock = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_HANG_HIGH_LOCK_IN, 0.8, true);
                Action hangSequence = new SequentialAction(
                        new ParallelAction(flipWristAway, lowerArmLow, new SequentialAction(lowerSlideLowFirst, lowerSlideLowSecond)), lowerArmLowLock, powerMotorsHold,
                        waitForLB1,
                        raiseSlidePrep, raiseArmPrep,
                        waitForLB2,
                        new ParallelAction(raiseArmSwing, new SequentialAction(new SleepAction(0.5), lowerSlideSwing)), powerMotorsHoldLock, lowerArmLock, new SleepAction(2), lowerSlideLock
                        );
                loopUpdater.addAction(hangSequence);
            }
            if (smartGamepad1.left_trigger_pressed()){ //outtake + drop and go down
                Action rollOuttake = sampleIntake.getStartAndStopRollerAction(false, false);
                Action turnWristDown = sampleIntake.getTurnWristAction(SampleIntake.WRIST_IDLE_ROLLER, false);
                Action waitForLT = smartGamepad1.getWaitForButtons("left_trigger", false);
                Action outtakeAndLeave = new SequentialAction(rollOuttake,
                        new ParallelAction(turnWristDown, new SleepAction(0)),
                        rotatingSlide.retractSlide());
                loopUpdater.addAction(outtakeAndLeave);
            }

            if(smartGamepad1.right_bumper_pressed()){ //intake specimen and move slightly up
                //if its currently closed, do open sequence and vise versa.
                if(specimenIntake.isClawOpen()){
                    Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.CLOSE, false);
                    Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP_IN, 0.4, false);
                    loopUpdater.addAction(new SequentialAction(closeSpecimen, raiseSlide));

                } else {
                    Action lowerSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE_IN, 0.4,  false);
                    Action openSpecimen= specimenIntake.getMoveSpecimenIntake(SpecimenIntake.OPEN, false);
                    Action lowerSlide2 = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT_IN, 1, true);
                    loopUpdater.addAction(new SequentialAction(lowerSlide, openSpecimen, new SleepAction(0.15), lowerSlide2));

                }


            } if (smartGamepad1.left_bumper_pressed()){
                Action openSpecimen= specimenIntake.getMoveSpecimenIntake(SpecimenIntake.OPEN, false);
                loopUpdater.addAction(openSpecimen);
            }


            if(gamepad2.dpad_up){
                rotatingSlide.slide.adjustLift(1 * Math.min(1, (gamepad2.right_trigger+MIN_SLOW_MODE)));
            }  if(gamepad2.dpad_down){
                rotatingSlide.slide.adjustLift(-1 * Math.min(1, (gamepad2.right_trigger+MIN_SLOW_MODE)));
            }  if(gamepad2.dpad_right){
                Action moveArm = rotatingSlide.arm.getArmToPosition(rotatingSlide.arm.getMotorPositionAngle() + (10 * Math.min(1, (gamepad2.right_trigger+MIN_SLOW_MODE))), false);
                loopUpdater.addAction(moveArm);
            }  if(gamepad2.dpad_left){
                Action moveArm = rotatingSlide.arm.getArmToPosition(rotatingSlide.arm.getMotorPositionAngle() - (10 * Math.min(1, (gamepad2.right_trigger+MIN_SLOW_MODE))), false);
                loopUpdater.addAction(moveArm);
            } if ((!gamepad2.dpad_down && !gamepad2.dpad_up) && rotatingSlide.slide.isLevelReached()){
                //Log.v("manualControl AATele slide holdPosition", "is target reached? " + rotatingSlide.slide.isLevelReached());
                Log.v("Slide Power", "invoking holdPosition at 3");
                rotatingSlide.slide.holdPosition();
            }
            //player 1 also controls slides, but only fine-tuning
            if(smartGamepad1.dpad_up_pressed()){
                //rotatingSlide.slide.adjustLift(MIN_SLOW_MODE);
                Action adjustSlide = rotatingSlide.slide.getSlideToPosition(rotatingSlide.slide.getPosition() + 3 * Math.min(1, (gamepad1.right_trigger+MIN_SLOW_MODE)), 1, false);
                loopUpdater.addAction(adjustSlide);
            }  if(smartGamepad1.dpad_down_pressed()){
                //rotatingSlide.slide.adjustLift(-1 * MIN_SLOW_MODE);
                Action adjustSlide = rotatingSlide.slide.getSlideToPosition(rotatingSlide.slide.getPosition() - 3 * Math.min(1, (gamepad1.right_trigger+MIN_SLOW_MODE)), 1, false);
                loopUpdater.addAction(adjustSlide);
            }  if(gamepad1.dpad_right){
                leftFrontPower = DPAD_DRIVE*1.2;
                rightFrontPower = -1 * DPAD_DRIVE;
                leftBackPower = -1 * DPAD_DRIVE;
                rightBackPower = DPAD_DRIVE;
            }  if(gamepad1.dpad_left){
                leftFrontPower = -1 * DPAD_DRIVE*1.2;
                rightFrontPower =  DPAD_DRIVE;
                leftBackPower = DPAD_DRIVE;
                rightBackPower = -1 * DPAD_DRIVE;

            } if ((!gamepad1.dpad_down && !gamepad1.dpad_up) && rotatingSlide.slide.isLevelReached()){
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




            if(smartGamepad2.left_stick_button_pressed() || smartGamepad1.y_pressed()){
                loopUpdater.clearActions();
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
                sampleIntake.manualMoveRoller(SampleIntake.OUTTAKE_POWER_ROLLER);
            } else if (smartGamepad2.left_trigger_released()){
                sampleIntake.manualMoveRoller(0);
            }


            // Send calculated power to wheels after accounting for dpad controls if needed
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            Twist2dDual<Time> twist = localizer.update();
            pose = pose.plus(twist.value());

            rotatingSlide.update();
            // Show the elapsed game time and wheel power.
            //telemetry.addD  ata("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("Slides Position", "left: "  + rotatingSlide.slide.getLeftEncoder() + "right: " + rotatingSlide.slide.getRightEncoder());
            telemetry.addData("Slides Position Inches", "left: "  + rotatingSlide.slide.getPosition());
            //telemetry.addData("slides velocity", "left: " + rotatingSlide.slide.getLeftVelocity() + "right: " + rotatingSlide.slide.getRightVelocity());
            telemetry.addData("worm position", ""+ rotatingSlide.arm.getMotorPositionAngle());
            telemetry.addData("wrist position", ""+ sampleIntake.getWristPosition());
            telemetry.addData("# Active Actions: ", loopUpdater.getActiveActions().size());
            telemetry.addData("Pose:", "x: %4.3f y: %4.3f", pose.position.x, pose.position.y);
            telemetry.addData("Pose:", "heading: " + pose.heading);
            telemetry.addData("End of arm location:", ""+ (pose.position.x + pose.heading.real*(rotatingSlide.getHorizontalExpansionLength()+robotLengthFromCOM)));
            telemetry.update();
            loopUpdater.updateAndRunAll();


        }
    }
}