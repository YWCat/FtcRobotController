package org.firstinspires.ftc.teamcode.utility;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class OnlyMoveArm extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotorEx armMotor = null;

    private int chamberLenTicks = 2000;
    private int restartTicks = 0;
    private double upPower = 1;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        armMotor  = hardwareMap.get(DcMotorEx.class, RobotConfig.armMotor);

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

        //armMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            armMotor.setTargetPosition(chamberLenTicks);
            if(armMotor.getCurrentPosition()<chamberLenTicks){
                armMotor.setPower(upPower);
                Log.i("armMotor", "motor pos: " + armMotor.getCurrentPosition());
            }
            else {
                armMotor.setPower(0);
                Log.i("Actions", "arm done lifting to chamber");
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("position", "%d", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}