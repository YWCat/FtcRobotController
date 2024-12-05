package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class JoystickMapTester extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx slideMotorL = null;
    private DcMotorEx slideMotorR = null;
    private DcMotor armMotor = null;
    private CRServo intakeServo;
    private Servo specimenServo;
    private Servo wrist;
    private boolean slowModeOn = false;

    @Override
    public void runOpMode() {
        RobotCore robotCore  = new RobotCore(this);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, RobotConfig.leftFront);
        leftBackDrive  = hardwareMap.get(DcMotor.class, RobotConfig.leftRear);
        rightFrontDrive = hardwareMap.get(DcMotor.class, RobotConfig.rightFront);
        rightBackDrive = hardwareMap.get(DcMotor.class, RobotConfig.rightRear);

        intakeServo = hardwareMap.crservo.get(RobotConfig.sampleServo);
        specimenServo = hardwareMap.get(Servo.class, RobotConfig.specimenServo);

        slideMotorL = hardwareMap.get(DcMotorEx.class, RobotConfig.slideMotorL);
        slideMotorR = hardwareMap.get(DcMotorEx.class, RobotConfig.slideMotorR);
        slideMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMotorR.setDirection(DcMotor.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotor.class, RobotConfig.armMotor);

        wrist = hardwareMap.get(Servo.class, RobotConfig.wrist);

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

            double joystickRadius = Math.min(1,Math.sqrt(Math.pow(gamepad1.left_stick_y,2) + Math.pow(gamepad1.left_stick_x,2)));
            lateral = robotCore.mapJsComponents(gamepad1.left_stick_x, joystickRadius, slowModeOn);
            axial = robotCore.mapJsComponents(-gamepad1.left_stick_y, joystickRadius, slowModeOn);
            yaw = robotCore.mapJsComponents(gamepad1.right_stick_x, gamepad1.right_stick_x, slowModeOn);


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
            if(gamepad1.dpad_left){
                slowModeOn = true;
            } else{
                slowModeOn = false;
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
            //leftFrontDrive.setPower(leftFrontPower);
            //rightFrontDrive.setPower(rightFrontPower);
            //leftBackDrive.setPower(leftBackPower);
            //rightBackDrive.setPower(rightBackPower);

            double factor = 0.982;
            if(gamepad2.dpad_up){
                //slideMotorL.setTargetPosition(2300);
                //slideMotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slideMotorL.setVelocity(2000);

                //slideMotorR.setTargetPosition(2300);
                //slideMotorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slideMotorR.setVelocity(2000 * factor);

                /*

                slideMotorL.setPower(0.5);
                slideMotorR.setPower(0.5);
                 */
            } else if(gamepad2.dpad_down){
                //slideMotorL.setTargetPosition(0);
                //slideMotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slideMotorL.setVelocity(-2000);

                //slideMotorR.setTargetPosition(0);
                //slideMotorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slideMotorR.setVelocity(-2000*factor);

            } else{
                slideMotorL.setVelocity(0);
                slideMotorR.setVelocity(0);
                slideMotorL.setPower(0.1);
                slideMotorR.setPower(0.1);
            }
            if(gamepad1.right_trigger > 0.5){
                wrist.setPosition(wrist.getPosition()+0.0025);
            }
            if(gamepad1.left_trigger > 0.5){
                wrist.setPosition(wrist.getPosition()-0.0025);
            }
            if(gamepad2.right_stick_button){
                slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
                slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad1.a){
                wrist.setPosition(0.8); //hold
            } if(gamepad1.b){
                wrist.setPosition(0.67); //intake
            } if(gamepad1.x){
                wrist.setPosition(0.9);//outtake
            } if (gamepad1.y){
                wrist.setPosition(0.1); //base?
            }
            if(gamepad2.a){
                intakeServo.setPower(1);
            } else if(gamepad2.b){
                intakeServo.setPower(-1);
            }else{
                intakeServo.setPower(0);
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Left stick X/Y: ", "X: "+ gamepad1.left_stick_x + " Y: " + gamepad1.left_stick_y);
            telemetry.addData("Right stick X: ", "R: "+ gamepad1.right_stick_x);
            telemetry.addData("Remapped power", "X: %4.2f  Y: %4.2f R: %4.2f", lateral, axial, yaw);
            telemetry.addData("Slides Position", "left: "  + slideMotorL.getCurrentPosition() + " right: " + slideMotorR.getCurrentPosition());
            telemetry.addData("worm position", ""+ armMotor.getCurrentPosition());
            telemetry.addData("intake position:", "%4.2f", wrist.getPosition());
            telemetry.update();
        }
    }
}