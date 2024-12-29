package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ArmServoTest", group="Linear OpMode")
public final class ArmServoTest extends LinearOpMode {
    public int intakeState = 0;
    boolean isFast = true;

    boolean handClosed = true;
    boolean wristUp = false;
    boolean slideControlled = false;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Allows for asynchronous operation
    public static void setTimeout(Runnable runnable, int delay) {
        new Thread(()-> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }
            catch (Exception e) {
                System.err.println(e);
            }
        }).start();
    }

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
            // servo 3: wrist, servo 4: hand
            //claw
            if (gamepad1.right_bumper) { // slightly raised position
                servo3.setPosition(0.6);
                wristUp = true;
            }
            if(gamepad1.left_bumper) { // up: 1, down: 0.5
                if (!wristUp) {
                    servo3.setPosition(1);
                }
                else {
                    servo3.setPosition(0.5);
                }
                wristUp = !wristUp;
                sleep(100);
            }
            if(gamepad1.y) { // open: 1, closed: 0.7
                if (!handClosed) {
                    servo4.setPosition(1);
                }
                else {
                    servo4.setPosition(0.7);
                }
                handClosed = !handClosed;
                sleep(100);
            }

            if (gamepad2.x) { // lower arm, swing down
                if (!slideControlled) {
                    slideControlled = true;

                    setTimeout(() -> {
                        servo3.setPosition(0.6);
                        sleep(100);

                        // retract arm
                        Lslide.setPower(-1);
                        Rslide.setPower(-1);
                        sleep(10);

                        // slides moving in negative direction
                        while (Lslide.getVelocity() < -3 || Rslide.getVelocity() < -3) {
                            /* wait */
                        }

                        Lslide.setPower(0);
                        Rslide.setPower(0);

                        // swing arm
                        wormMotor.setTargetPosition(intakePos);
                        wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wormMotor.setPower(1);

                        slideControlled = false;
                    }, 0);
                }
            }

            if (gamepad2.y) { // retract arm, swing up, and re-extend
                if (!slideControlled) {
                    slideControlled = true;

                    // long sequence, run asynchronously
                    setTimeout(()->{
                        // retract arm
                        Lslide.setPower(-1);
                        Rslide.setPower(-1);
                        sleep(10);

                        // slides moving in negative direction
                        while (Lslide.getVelocity() < -3 || Rslide.getVelocity() < -3) {
                            /* wait */
                        }
                        Lslide.setPower(0);
                        Rslide.setPower(0);

                        // swing arm
                        wormMotor.setTargetPosition(slidePos);
                        wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wormMotor.setPower(1);

                        while (Math.abs(wormMotor.getCurrentPosition() - wormMotor.getTargetPosition()) > 10) {
                            /* wait */
                        }

                        // extend arm
                        Lslide.setPower(1);
                        Rslide.setPower(1);
                        sleep(10);

                        // slides moving in positive direction
                        while (Lslide.getVelocity() > 10 || Rslide.getVelocity() > 10) {
                            /* wait */
                        }
                        Lslide.setPower(0);
                        Rslide.setPower(0);

                        slideControlled = false;
                    }, 0);
                }
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
                    slideControlled = false;
                    Lslide.setPower(1);
                    Rslide.setPower(1);
                } else if (gamepad1.dpad_down) {
                    slideControlled = false;
                    Lslide.setPower(-1);
                    Rslide.setPower(-1);
                } else if (!slideControlled) {
                    Lslide.setPower(0);
                    Rslide.setPower(0);
                }
            }
            else{
                if (gamepad1.dpad_up) {
                    slideControlled = false;
                    Lslide.setPower(0.5);
                    Rslide.setPower(0.5);
                } else if (gamepad1.dpad_down) {
                    slideControlled = false;
                    Lslide.setPower(-0.5);
                    Rslide.setPower(-0.5);
                } else if (!slideControlled) {
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