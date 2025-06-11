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

@TeleOp(name="Drive", group="Linear OpMode")
public class ADriveOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeMotor = null;
    private Servo armServo = null;
    private Servo clawServo = null;
    private Servo dumpServo;
    private double clawOpenPos = 0.65;
    private double clawAdjustPos = 0.75;
    private double clawClosePos = 0.9;
    private double clawTargetPos = 0.65;
    private double armUpPos = 0.55;
    private double armTransportPos = 0.25;
    private double armDwnDropPos = 0.22;
    private double armDwnPickPos = 0.015;
    private double armDwnPlacePos = 0.0;
    private double armTargetPos = 0.55;
    private int armState;

    //slide algorithm variables
    private double slideTargetPos = 0;
    private double Kp1 = 1;
    private double Kp2 = 1;
    private double sweep_time;
    private double dump_time;
    private double grabbedTime;
    private double openedTime;
    private double openedTime2;
    private boolean intaking = false;

    @Override
    public void runOpMode() {
        armState = 0;
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        SlideSubsystem Slides = new SlideSubsystem(hardwareMap);
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
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
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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
            leftFrontDrive.setPower(leftFrontPower*0.605);
            rightFrontDrive.setPower(rightFrontPower*0.605);
            leftBackDrive.setPower(leftBackPower*0.605);
            rightBackDrive.setPower(rightBackPower*0.605);
            //The slides will always be raised to what the targetPos is set as
            Slides.raiseSlide(slideTargetPos);
            //armServo will always go to the armTargetPos, which can be adjusted
            armServo.setPosition(armTargetPos);
            //clawServo will always go to the clawTargetPos, either closed or open
            clawServo.setPosition(clawTargetPos);
            //Section: Arm state machine
            //Checks if gamepad1.a is pressed and if the current state of the arm is zero(default)
            if(gamepad1.a && armState==0){
                //Tells the drivers that the arm is going to be ready to intake
                telemetry.addLine("Going to intake position");
                //Sets the arm to the down position
                armTargetPos = armDwnPlacePos;
                //Registers the current state of the arm as 1(ready to grab)
                armState = 1;
            }
            //Checks if the arm is ready to grab and that gamepad1.x is pressed
            if(armState == 1 && gamepad1.x){
                //Tells the driver that the arm is currently intaking
                telemetry.addLine("Closing Claw");
                //Raises the arm to a height where it can drop the cube
                armTargetPos = armDwnPickPos;
                //Closes the claw while the arm is traveling up
                clawTargetPos = clawClosePos;
                //Creates a timestamp of when the claw has closed
                grabbedTime = System.currentTimeMillis();
                //Moves on to the next state(transport)
                armState = 2;
            }
            //In case the first time we intake doesn't work
            //Checks if 0.5 seconds has elapsed after the claw has closed
            if(System.currentTimeMillis() - grabbedTime >= 500 && armState == 2){
                telemetry.addLine("Arm going Up");
                //Sets the arm to the up position
                armTargetPos = armTransportPos;
                //Registers the current state of the arm as 3(transporting)
                armState = 3;
            }
            // Checks if the arm is currently in transport and if gamepad1.a is pressed again
            if(armState == 3 && gamepad1.a){
                //Sets the arm to the down position
                armTargetPos = armDwnPlacePos;
                //Closes the claw
                clawTargetPos = clawOpenPos;
                //Registers the current state of the arm as 1(ready to grab), resetting the claw
                armState = 1;
            }
            //If gamepad1.x is pressed and the state of the arm is 3(transporting)
            if(gamepad1.x && armState == 3){
                //The arm goes down a little bit
                armTargetPos = armDwnDropPos;
                //The claw opens
                clawTargetPos = clawOpenPos;
                //The timestamp of when the claw opens is taken
                openedTime = System.currentTimeMillis();
                //Advance to next state(which is the cube dropped pos)
                armState = 10;
            }
            //If the time since the claw opened is over half a second, and the armState is 10, then the arm can go up
            if(System.currentTimeMillis() - openedTime >= 500 && armState == 10) {
                armTargetPos = armUpPos;
                armState = 0;
            }
            //Checks to see if gamepad1.b is pressed and that the robot is currently transporting something
            if(gamepad1.b && (armState == 3)){
                telemetry.addLine("Arm going down");
                //Sets the arm to the down position
                armTargetPos = armDwnPlacePos;
                //Registers the state of the subsystem as 4(opening)
                armState = 4;
            }
            //Checks to see if the claw is ready to open and if gamepad1.x is pressed
            if(armState == 4 && gamepad1.x){
                telemetry.addLine("Opening Claw");
                //Opens the claw slightly
                clawTargetPos = clawAdjustPos;
                //Creates a timestamp of when the claw opened
                openedTime = System.currentTimeMillis();
                armState = 5;
            }
            //Checks if 0.5 seconds have elapsed since the claw has been opened
            if(System.currentTimeMillis() - openedTime >= 500 && armState == 5) {
                clawTargetPos = clawOpenPos;
                openedTime2 = System.currentTimeMillis();
                armState = 6;
            }
            if(System.currentTimeMillis() - openedTime2 >= 1500 && armState == 6){
                telemetry.addLine("Arm going up + reset");
                //Moves the arm back up
                armTargetPos = armUpPos;
                //Sets the state of the arm to 0(default position)
                armState = 0;
            }


            //These two functions are for the slide. They set the targetPos, which the slides are constantly going towards
            //IMPORTANT: if you want to adjust the height the slide goes to, remember the targetPos is in TICKS
            //You may run the DualMotorOpMode to find out the current position of the slide motors, as well as adjust it little by little
            if(gamepad1.dpad_up && !intaking){
                slideTargetPos = 2425;
            }
            if(gamepad1.dpad_right){
                slideTargetPos = 388;
            }
            if(gamepad1.dpad_down){
                slideTargetPos = 0;
            }

            //This function does two things:
            //1. When y is pressed, the dumpServo is set to the open position
            //2. A timer is set, keeping track of the amount of time since y was last pressed in the variable dump_time
            if(gamepad1.y){
                dumpServo.setPosition(0.0);
                dump_time = System.currentTimeMillis();
            }
            //If it has been more than 1 second since y was pressed, the dumpServo goes to the close position by default
            if(System.currentTimeMillis()-dump_time > 1500){
                dumpServo.setPosition(0.365);
            }

            //This works in a similar manner to the dumpServo
            //The intakeMotor starts intaking, and the time since right_bumper was last pressed is noted in rb_time
            if(gamepad1.right_bumper){
                intakeMotor.setPower(-0.4);
                intaking = true;
                sweep_time = System.currentTimeMillis();
            }
            if(gamepad1.left_bumper){
                intakeMotor.setPower(0.4);
                intaking = true;
                sweep_time = System.currentTimeMillis();
            }
            //If it has been more than 0.8 seconds since right_bumper was pressed, the intakeMotor is set at 0 power
            if(System.currentTimeMillis()- sweep_time > 1000) {
                intaking = false;
                intakeMotor.setPower(0);
            }
            // Quite shrimple, just tells you the information listed in the caption
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm State:", armState);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}