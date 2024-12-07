package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.RobotCore;

public class LimeLight {

    private Limelight3A limelight;
    private Pose3D botpose;
    public double timeStamp; // in s
    public Pose2d limePose;
    public double xOffset = 0; //X correction, in inch
    public double yOffset = 0; //Y correction, in inch
    public LimeLight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
    }
    public void getLLPose() {

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
            botpose = result.getBotpose();
            timeStamp = System.nanoTime() * 1e-9;
            limePose = new Pose2d(
                    botpose.getPosition().x * 100 / 2.54 + xOffset, //LL in meter, convert to inch
                    botpose.getPosition().y * 100 / 2.54 + yOffset,
                    Math.toRadians(botpose.getOrientation().getYaw()) // LL yaw in degrees
            );
            Log.v("LLCam-CC", "Pose3d X=" +botpose.getPosition().x
                                   + " Y=" +botpose.getPosition().y
                    +" YAW="+botpose.getOrientation().getYaw());
            Log.v("LLCam-CC", "Pose2D X=" +limePose.position.x
                  + " Y=" + limePose.position.y + " H="+limePose.heading);
        } else {
            limePose = null;
        }
    }
}
