/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Drive", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeMotor = null;
    private Servo clawLeftServo = null;
    private Servo dumpServo;
    private double clawLeftOpen_pos = 0.6;//0.32 to grab, 1.0 to open
    private double clawLeftClose_pos = 0.32;
    private double clawLeftServo_pos = 0.34;
    private Servo clawRightServo = null;
    private double clawRightOpen_pos = 0.25;//0.25
    private double clawRightClose_pos = 0.6;//0.6
    private double clawRightServo_pos = 0.2;//0,
    //slide algorithm variables
    private int targetPos = 0;
    private double Kp1 = 1;
    private double Kp2 = 1;
    private double rb_time;
    private double dump_time;
    private DcMotor linearSlide1;
    private DcMotor linearSlide2 ;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        SlideSubsystem Slides = new SlideSubsystem(hardwareMap);
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        dumpServo = hardwareMap.get(Servo.class, "dumpServo");

        //intakeMotor = hardwareMap.get(DcMotor.class,"eater");
        //clawLeftServo = hardwareMap.get(Servo.class,"clawLeftServo");
        //clawRightServo = hardwareMap.get(Servo.class,"clawRightServo");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            //IMPORTANT: if you want to adjust how fast the robot is, change how much the power is divided by -B
            leftFrontDrive.setPower(-leftFrontPower/2);
            rightFrontDrive.setPower(-rightFrontPower/2);
            leftBackDrive.setPower(-leftBackPower/2);
            rightBackDrive.setPower(-rightBackPower/2);
            //The slides will always be raised to what the targetPos is set as
            Slides.raiseSlide(targetPos);
            //commented out code, might be useful later
            /*
            if(gamepad1.dpad_up){
                clawLeftServo_pos+=0.05;
                clawLeftServo.setPosition(clawLeftOpen_pos);
                sleep(300);
                //clawLeftServo.setPosition(clawLeftClose_pos);
            }
            if(gamepad1.dpad_down){
                clawLeftServo_pos-=0.05;
                clawLeftServo.setPosition(clawLeftClose_pos);
                sleep(300);
                //clawLeftServo.setPosition(clawLeftOpen_pos);
            }

            if(gamepad1.dpad_left){
                clawRightServo_pos+=0.05;
                clawRightServo.setPosition(clawRightOpen_pos);
                sleep(300);
                //clawRightServo.setPosition(clawRightOpen_pos);
            }
            if(gamepad1.dpad_right){
                clawRightServo_pos-=0.05;
                clawRightServo.setPosition(clawRightClose_pos);
                sleep(300);
                //clawRightServo.setPosition(clawRightClose_pos);
            }
             */
            //These two functions are for the slide. They set the targetPos, which the slides are constantly going towards
            //IMPORTANT: if you want to adjust the height the slide goes to, remember the targetPos is in TICKS
            //You may run the DualMotorOpMode to find out the current position of the slide motors, as well as adjust it little by little
            if(gamepad1.dpad_up){
                targetPos = 2425;
            }
            if(gamepad1.dpad_down){
                targetPos = 0;
            }

            //This function does two things:
            //1. When y is pressed, the dumpServo is set to the open position
            //2. A timer is set, keeping track of the amount of time since y was last pressed in the variable dump_time
            if(gamepad1.y){
                dumpServo.setPosition(0.0);
                dump_time = System.currentTimeMillis();
            }
            //If it has been more than 1 second since y was pressed, the dumpServo goes to the close position by default
            if(System.currentTimeMillis()-dump_time > 1000){
                dumpServo.setPosition(0.355);
            }

            //This works in a similar manner to the dumpServo
            //The intakeMotor starts intaking, and the time since right_bumper was last pressed is noted in rb_time
            if(gamepad1.right_bumper){
                intakeMotor.setPower(-0.6);
                rb_time = System.currentTimeMillis();
            }
            //If it has been more than 0.8 seconds since right_bumper was pressed, the intakeMotor is set at 0 power
            if(System.currentTimeMillis()- rb_time > 800) {
                intakeMotor.setPower(0);
            }
            // Quite shrimple, just tells you the information listed in the caption
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}