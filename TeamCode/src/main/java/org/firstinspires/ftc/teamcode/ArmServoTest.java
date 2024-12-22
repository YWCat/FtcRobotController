package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="ArmServoTest", group="Linear OpMode")
public final class ArmServoTest extends LinearOpMode {
    public int intakeState = 0;
    boolean isFast = true;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    @Override
    public void runOpMode() throws InterruptedException {
        double servoPos = 0;
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo servo3 = hardwareMap.get(Servo.class, "servo3"); //base:1.0, intake: 0.45, outtake: 1.0
        Servo servo4 = hardwareMap.get(Servo.class, "servo4"); //intake:0.6 outtake:0.7

        //Worm Motor
        DcMotorEx wormMotor = hardwareMap.get(DcMotorEx.class, "motor");//intake: 2789, outtake: 200
        int intakePos = 2789;
        int slidePos = 0;
        int tempPos;
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setTargetPosition(intakePos);
        wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Slide

        DcMotorEx Lslide = hardwareMap.get(DcMotorEx.class,"slideL");
        DcMotorEx Rslide = hardwareMap.get(DcMotorEx.class,"slideR");
        Lslide.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (!isStopRequested()) {
            //claw
            if(gamepad1.x){
                servo3.setPosition(0.45);
                servo4.setPosition(0.5);
            }
            if(gamepad1.y){
                servo3.setPosition(1.0);
                servo4.setPosition(0.7);
            }
            //move
            double max;
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
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            //Wormgear
            if(gamepad2.a){
                wormMotor.setTargetPosition(slidePos);
                wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormMotor.setPower(0.5);
            }
            if(gamepad2.b){
                wormMotor.setTargetPosition(intakePos);
                wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormMotor.setPower(0.5);
            }
            if(gamepad2.dpad_down){
                if(wormMotor.getCurrentPosition()-100 <= -200){
                    tempPos = -200;
                }
                else{
                    tempPos = wormMotor.getCurrentPosition()-100;
                }
                wormMotor.setTargetPosition(tempPos);
                wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormMotor.setPower(0.15);
            }
            if(gamepad2.dpad_up){
                if(wormMotor.getCurrentPosition()+100 <= 2789){
                    tempPos = 2789;
                }
                else{
                    tempPos = wormMotor.getCurrentPosition()+100;
                }
                wormMotor.setTargetPosition(tempPos);
                wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormMotor.setPower(0.1);
            }
            //Slide controls
            if(gamepad1.b){
                isFast = !isFast;
                telemetry.addData("b is pressed?", isFast);
            }
            if(isFast) {
                if (gamepad1.dpad_up) {
                    Lslide.setPower(1);
                    Rslide.setPower(1);
                } else if (gamepad1.dpad_down) {
                    Lslide.setPower(-1);
                    Rslide.setPower(-1);
                } else{
                    Lslide.setPower(0);
                    Rslide.setPower(0);
                }
            }
            else{
                if (gamepad1.dpad_up) {
                    Lslide.setPower(0.5);
                    Rslide.setPower(0.5);
                } else if (gamepad1.dpad_down) {
                    Lslide.setPower(-0.5);
                    Rslide.setPower(-0.5);
                } else{
                    Lslide.setPower(0);
                    Rslide.setPower(0);
                }
            }
            telemetry.addData("Worm Motor Encoder Reading: ", wormMotor.getCurrentPosition());
            telemetry.update();
            sleep(100);
        }
    }
}