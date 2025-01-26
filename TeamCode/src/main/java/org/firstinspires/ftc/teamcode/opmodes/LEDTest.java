package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightColor;
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SampleIntakeClaw;
import org.firstinspires.ftc.teamcode.utility.LoopUpdater;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;
import org.firstinspires.ftc.teamcode.utility.SmartGamepad;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp
public class LEDTest extends LinearOpMode {
    private SmartGamepad smartGamepad1 = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        RobotCore robotCore  = new RobotCore(this);
        Servo LED = robotCore.hardwareMap.get(Servo.class, RobotConfig.ledLight);
        double brightness = 0.5;
        LoopUpdater loopUpdater = new LoopUpdater();
        smartGamepad1 = new SmartGamepad(gamepad1);

        telemetry.setMsTransmissionInterval(11);
        double intake = SampleIntakeClaw.WRIST_INTAKE_CLAW;

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        LED.setPosition(brightness);
        waitForStart();

        while (opModeIsActive()) {
            if (smartGamepad1.left_bumper_pressed()) {
                brightness += 0.1;
                LED.setPosition(brightness);
                telemetry.addData("brightness = %0f", brightness);
            }
            if (smartGamepad1.left_trigger_pressed()) {
                brightness -= 0.1;
                LED.setPosition(brightness);
                telemetry.addData("brightness = %0f", brightness);
            }
            telemetry.update();
            loopUpdater.updateAndRunAll();
        }
    }
}