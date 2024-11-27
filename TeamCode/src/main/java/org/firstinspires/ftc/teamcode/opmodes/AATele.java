package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //TODO: replace this with smartGamepad
    private SmartGamepad smartGamepad1 = null;
    private SmartGamepad smartGamepad2 = null;
    private static final double MIN_SLOW_MODE = 0.3;
    private static final double DRIVE_SLOW_MODE = 0.3;
    private boolean slowModeOn = true;
    private  boolean specimenOpenToggle = false;


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
        leftFrontDrive  = hardwareMap.get(DcMotor.class, RobotConfig.leftFront);
        leftBackDrive  = hardwareMap.get(DcMotor.class, RobotConfig.leftRear);
        rightFrontDrive = hardwareMap.get(DcMotor.class, RobotConfig.rightFront);
        rightBackDrive = hardwareMap.get(DcMotor.class, RobotConfig.rightRear);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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

            double joystickRadius = Math.min(1,Math.sqrt(Math.pow(gamepad1.left_stick_y,2) + Math.pow(gamepad1.left_stick_x,2)));
            lateral = robotCore.mapJsComponents(gamepad1.left_stick_x, joystickRadius, slowModeOn);
            axial = robotCore.mapJsComponents(-gamepad1.left_stick_y, joystickRadius, slowModeOn);


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
            if(gamepad1.left_stick_button){
                slowModeOn = true;
            } else{
                slowModeOn = false;
            }


            if(smartGamepad2.x_pressed()){ //retract slides and make arm vertical
                loopUpdater.addAction(rotatingSlide.retractSlide());
                Log.i("UpdateActions", "Added retract action; Actions Active: " +  loopUpdater.getActiveActions().size());
            } else if (smartGamepad2.b_pressed()){ // go to chamber
                Action armToChamber = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_CHAMBER_PREP,false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP, true);
                Action toChamber = new SequentialAction(retractSlide, armToChamber, extendSlide);
                loopUpdater.addAction(toChamber);
                Log.i("UpdateActions", "Actions Active: " +  loopUpdater.getActiveActions().size());
            }

            if (smartGamepad2.a_pressed()){ // put everything in position to intake
                Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE, false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_INTAKE, true);
                Action prepIntake = sampleIntake.getPrepIOAction(true, false);

                Action toIntake = new ParallelAction(prepIntake, new SequentialAction(retractSlide, armToIntake, extendSlide));
                //Action toIntake = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_INTAKE, true);
                loopUpdater.addAction(toIntake);
                Log.i("UpdateActions", "Intake prep added;  Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad2.y_pressed()){ //basket outtake prep

                Action armToBasket = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_BASKET,false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET, true);
                Action closeClaw = sampleIntake.getStartRollerAction(true, false);
                Action turnWrist = sampleIntake.getTurnWristAction(SampleIntake.WRIST_OUTTAKE_CLAW, false);
                Action toOuttake = new ParallelAction(turnWrist, new SequentialAction( retractSlide, armToBasket , extendSlide));
                loopUpdater.addAction(toOuttake);
                Log.i("UpdateActions", "Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad1.right_trigger_pressed()){ //start rollers INTAKE or pick up and go
                Action rollIntake = sampleIntake.getStartRollerAction(true, false);
                Action turnWristDown = sampleIntake.getTurnWristAction(SampleIntake.WRIST_IDLE_ROLLER, false);
                Action outtakeAction = new SequentialAction(rollIntake, turnWristDown);
                Log.i("UpdateActions", "start intake; Actions Active: " +  loopUpdater.getActiveActions().size());
                loopUpdater.addAction(outtakeAction);
            }
            if (smartGamepad1.left_trigger_pressed()){ //outtake + drop and go down
                Action rollOuttake = sampleIntake.getStartRollerAction(false, false);
                Action turnWristDown = sampleIntake.getTurnWristAction(SampleIntake.WRIST_IDLE_ROLLER, false);
                Action outtakeAndLeave = new SequentialAction(rollOuttake, turnWristDown);
                if(!smartGamepad1.dpad_up){ //manual mode turns this off
                    outtakeAndLeave = new SequentialAction(rollOuttake, turnWristDown,
                            rotatingSlide.retractSlide()
                    );
                }
                loopUpdater.addAction(outtakeAndLeave);
            }
            if(smartGamepad1.right_bumper_pressed()){ //intake specimen and move slightly up
                Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.CLOSE, false);
                if(!smartGamepad1.dpad_up){ //manual mode turns this off
                    Action raiseSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_PICK_UP_SPECIMEN, false);
                    loopUpdater.addAction(new SequentialAction(closeSpecimen, raiseSlide));
                } else {
                    loopUpdater.addAction(closeSpecimen);
                }

            } if (smartGamepad1.left_bumper_pressed()){
                Action openSpecimen = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.OPEN, false);
                if(!smartGamepad1.dpad_up){ //manual mode turns this off
                    Action lowerSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PLACE, false);
                    loopUpdater.addAction(new SequentialAction(lowerSlide, openSpecimen));
                } else{
                    loopUpdater.addAction(openSpecimen);
                }
            }

            if(gamepad2.dpad_up){
                Log.v("AAtele slide", "slide up detected");
                rotatingSlide.slide.adjustLift(1 * Math.min(1, (1-gamepad2.right_trigger+MIN_SLOW_MODE)), false);
            } else if(gamepad2.dpad_down){
                Log.v("AAtele slide", "slide down detected");
                rotatingSlide.slide.adjustLift(-1 * Math.min(1, (1-gamepad2.right_trigger+MIN_SLOW_MODE)), false);
            } else if(gamepad2.dpad_right){
                Log.v("AAtele arm", "arm down detected");
                Action moveArm = rotatingSlide.arm.getArmToPosition(rotatingSlide.arm.getMotorPosition() + (int) (250 * Math.min(1, (1-gamepad2.right_trigger+MIN_SLOW_MODE))), false);
                loopUpdater.addAction(moveArm);
            } else if(gamepad2.dpad_left){
                Log.v("AAtele arm", "arm up detected");
                Action moveArm = rotatingSlide.arm.getArmToPosition(rotatingSlide.arm.getMotorPosition() - (int) (250 * Math.min(1, (1-gamepad2.right_trigger+MIN_SLOW_MODE))), false);
                loopUpdater.addAction(moveArm);
            }else if ((!gamepad2.dpad_down && !gamepad2.dpad_up) && rotatingSlide.slide.isLevelReached()){
                //Log.v("manualControl AATele stopMotor", "is target reached? " + rotatingSlide.slide.isLevelReached());
                rotatingSlide.slide.stopMotor();
            }

            if(smartGamepad2.left_bumper_pressed()){
                double pos = sampleIntake.getWristPosition();
                Action wristIntake = sampleIntake.getTurnWristAction(pos + 0.025, false);
                loopUpdater.addAction(wristIntake);
            }
            if(smartGamepad2.right_bumper_pressed()){
                double pos = sampleIntake.getWristPosition();
                Action wristIntake = sampleIntake.getTurnWristAction(pos - 0.025, false);
                loopUpdater.addAction(wristIntake);
            }




            if(smartGamepad2.left_stick_button_pressed()){
                loopUpdater.clearActions();
            }
            if(smartGamepad2.right_stick_button_pressed()){
                rotatingSlide.slide.resetEncoder();
                rotatingSlide.arm.resetEncoder();
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            rotatingSlide.update();
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Slides Position", "left: "  + rotatingSlide.slide.getLeftEncoder() + "right: " + rotatingSlide.slide.getRightEncoder());
            telemetry.addData("slides velocity", "left: " + rotatingSlide.slide.getLeftVelocity() + "right: " + rotatingSlide.slide.getRightVelocity());
            telemetry.addData("worm position", ""+ rotatingSlide.arm.getMotorPosition());
            telemetry.addData("# Active Actions: ", loopUpdater.getActiveActions().size());
            telemetry.update();
            loopUpdater.updateAndRunAll();


        }
    }
}