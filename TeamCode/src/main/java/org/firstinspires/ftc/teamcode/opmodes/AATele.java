package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.utility.LoopUpdater;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;
import org.firstinspires.ftc.teamcode.utility.SmartGamepad;

import java.util.ArrayList;

@TeleOp
@Config
public class AATele extends LinearOpMode{
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx armMotor = null;
    private Servo claw = null;

    //TODO: replace this with smartGamepad
    private SmartGamepad smartGamepad1 = null;
    private SmartGamepad smartGamepad2 = null;

    private ArrayList<Action> activeActions = new ArrayList<Action>();


    @Override
    public void runOpMode() {
        LoopUpdater loopUpdater = new LoopUpdater(); //I'm not sure if I'm doing it right
        RotatingSlide rotatingSlide = new RotatingSlide();
        RobotCore robotCore  = new RobotCore(this);
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

            if (smartGamepad1.a_pressed()){
                Action armToChamber = rotatingSlide.arm.getArmToPosition(RotatingSlide.ARM_CHAMBER_PREP,false));
                Action toChamber = new SequentialAction(/* retract slide, */armToChamber /*, extend slide*/);
                loopUpdater.addAction(toChamber);
                Log.i("UpdateActions", "Actions Active: " + activeActions.size());
            }
            if (smartGamepad1.b_pressed()){
                Action armToIntake = rotatingSlide.rotSlideToPosition(RotatingSlide.ARM_INTAKE, RotatingSlide.SLIDE_CHAMBER_PLACE);
                Action toIntake = new SequentialAction(/* retract slide, */armToIntake /*, extend slide*/);
                loopUpdater.addAction(toIntake);
                Log.i("UpdateActions", "Actions Active: " + activeActions.size());
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
            telemetry.addData("# Active Actions: ", activeActions.size() );
            telemetry.update();
            loopUpdater.updateAndRunAll();


        }
    }
}