package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightColor;
import org.firstinspires.ftc.teamcode.subsystems.RotatingSlide;
import org.firstinspires.ftc.teamcode.subsystems.SampleIntakeClaw;
import org.firstinspires.ftc.teamcode.utility.LoopUpdater;
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
public class ZZTest extends LinearOpMode {

    private Limelight3A limelight;
    private SmartGamepad smartGamepad1 = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        RobotCore robotCore  = new RobotCore(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        LimeLightColor LLCamClr = new LimeLightColor(hardwareMap);
        //LLCam.xOffset = pos_multiplier*botLengthHalf;
        drive.LLCamClr = LLCamClr;
        RotatingSlide rotatingSlide = new RotatingSlide();
        SampleIntakeClaw sampleIntake = new SampleIntakeClaw();
        LoopUpdater loopUpdater = new LoopUpdater();
        smartGamepad1 = new SmartGamepad(gamepad1);

        telemetry.setMsTransmissionInterval(11);
        double intake = SampleIntakeClaw.WRIST_INTAKE_CLAW;

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (smartGamepad1.dpad_left_pressed()) {
                drive.setCamCorr(true, 1, 0, 0,10, 2);
                drive.alignByCam(true);
                drive.setCamCorr(false, 1, 0, 0, 10, 2);
            }
            if (smartGamepad1.dpad_right_pressed()) {
                drive.setCamCorr(true, 1, 0, 10, 10, 2);
                drive.alignByCam(true);
                drive.setCamCorr(false, 1, 0, 10, 10, 2);
            }
            if (smartGamepad1.dpad_up_pressed()) {
                drive.setCamCorr(true, 0.5, 1,0, -7, 2);
                drive.alignByCam(false);
                drive.setCamCorr(false, 0.5, 1,0, -7, 2);
            }
            if (smartGamepad1.dpad_down_pressed()) {
                drive.setCamCorr(true, 0.5, 1,10, -15, 2);
                drive.alignByCam(false);
                drive.setCamCorr(false, 0.5, 1,10, -15, 2);
            }
            if (gamepad1.a) {
                telemetry.addData("tx", LLCamClr.getTxTyTa(1, 0, 1));
                telemetry.addData("ty", LLCamClr.getTxTyTa(1, 1, 1));
            }
            if (gamepad1.b) {
                drive.setCamCorr(true, 1, 1,-1, -6, 2);
                drive.alignByCam(true);
                drive.alignByCam(false);
                drive.disCamCorr();
            }

            if (smartGamepad1.x_pressed()) {
                Action wristOutPrep = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_OUTTAKE_CLAW, true);
                loopUpdater.addAction(wristOutPrep);
            }
            if (smartGamepad1.y_pressed()) {
                Action wristIntake = sampleIntake.getTurnWristAction(SampleIntakeClaw.WRIST_INTAKE_CLAW, true);
                loopUpdater.addAction(wristIntake);
            }
            if (smartGamepad1.right_bumper_pressed()) {
                intake -= 0.02;
                Action wristIntakeInc = sampleIntake.getTurnWristAction(intake, true);
                loopUpdater.addAction(wristIntakeInc);
                telemetry.addData("wrist = %0f", intake);
            }
            telemetry.update();
            loopUpdater.updateAndRunAll();
        }
    }
}