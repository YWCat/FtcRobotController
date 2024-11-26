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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //TODO: replace this with smartGamepad
    private SmartGamepad smartGamepad1 = null;
    private SmartGamepad smartGamepad2 = null;
    private boolean specimenIsOpen = false;


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

            if (smartGamepad1.x_pressed()){ // go to chamber
                Action armToChamber = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_CHAMBER_PREP,false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_CHAMBER_PREP, false);
                Action toChamber = new SequentialAction(/* retract slide, */armToChamber /*, extend slide*/);
                loopUpdater.addAction(toChamber);
                Log.i("UpdateActions", "Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if (smartGamepad1.b_pressed()){ // put everything in position to intake
                Action armToIntake = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_INTAKE, false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_INTAKE, true);
                Action openClaw = sampleIntake.getStartRollerAction(false, false); //comment this out when switching to rollers
                Action turnWrist = sampleIntake.getTurnWristAction(SampleIntake.WRIST_INTAKE, false);
                Action toIntake = new ParallelAction(openClaw, turnWrist, new SequentialAction(retractSlide, armToIntake, extendSlide));
                loopUpdater.addAction(toIntake);
                Log.i("UpdateActions", "Intake prep added;  Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad1.y_pressed()){ //basket outtake prep
                Action armToBasket = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_BASKET,false);
                Action retractSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_RETRACT, false);
                Action extendSlide = rotatingSlide.slide.getSlideToPosition(RotatingSlide.SLIDE_BASKET, true);
                Action closeClaw = sampleIntake.getStartRollerAction(true, false);
                Action turnWrist = sampleIntake.getTurnWristAction(SampleIntake.WRIST_OUTTAKE, false);
                Action toIntake = new ParallelAction(turnWrist, new SequentialAction(/* retract slide, */armToBasket /*, extend slide*/));
                loopUpdater.addAction(toIntake);
                Log.i("UpdateActions", "Actions Active: " +  loopUpdater.getActiveActions().size());
            }
            if(smartGamepad1.left_bumper_pressed()){ //start rollers INTAKE or pick up and go
                Action rollIntake = sampleIntake.getStartRollerAction(true, false);
                Action turnWristDown = sampleIntake.getTurnWristAction(SampleIntake.WRIST_IDLE, false);
                Action intakeAndLeave = new SequentialAction(rollIntake, turnWristDown);
                loopUpdater.addAction(intakeAndLeave);
            }
            if (smartGamepad1.right_bumper_pressed()){ //outtake + drop and go
                Action rollOuttake = sampleIntake.getStartRollerAction(false, false);
                Action turnWristDown = sampleIntake.getTurnWristAction(SampleIntake.WRIST_IDLE, false);
                Action outtakeAndLeave = new SequentialAction(rollOuttake, turnWristDown);
                loopUpdater.addAction(outtakeAndLeave);
            }
            if(smartGamepad1.left_trigger_pressed()){
                if(specimenIsOpen){ //close specimen
                    Action closeSpecimen = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.CLOSE, false);
                    loopUpdater.addAction(closeSpecimen);
                } else{
                    Action openSpecimen = specimenIntake.getMoveSpecimenIntake(SpecimenIntake.OPEN, false);
                    loopUpdater.addAction(openSpecimen);
                }
            }

            if(gamepad2.dpad_up){
                Log.v("AAtele slide", "slide up detected");
                Action moveSlide = rotatingSlide.slide.getSlideToPosition(rotatingSlide.slide.getMotorLPosition() + 50, false);
                loopUpdater.addAction(moveSlide);
            } else if(gamepad2.dpad_down){
                Log.v("AAtele slide", "slide down detected");
                Action moveSlide = rotatingSlide.slide.getSlideToPosition(rotatingSlide.slide.getMotorLPosition() - 50, false);
                loopUpdater.addAction(moveSlide);
            }

            if(gamepad2.dpad_right){
                Log.v("AAtele arm", "arm down detected");
                Action moveArm = rotatingSlide.arm.getArmToPosition(rotatingSlide.arm.getMotorPosition() + 50, false);
                loopUpdater.addAction(moveArm);
            } else if(gamepad2.dpad_left){
                Log.v("AAtele arm", "arm up detected");
                Action moveArm = rotatingSlide.arm.getArmToPosition(rotatingSlide.arm.getMotorPosition() - 50, false);
                loopUpdater.addAction(moveArm);
            }



            if(smartGamepad1.right_trigger_pressed()){
                loopUpdater.clearActions();
                //TODO: stop the motions of each action
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);



            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Slides Position", "left: "  + rotatingSlide.slide.getMotorLPosition());
            telemetry.addData("worm position", ""+ rotatingSlide.arm.getMotorPosition());
            telemetry.addData("# Active Actions: ", loopUpdater.getActiveActions().size());
            telemetry.update();
            loopUpdater.updateAndRunAll();


        }
    }
}