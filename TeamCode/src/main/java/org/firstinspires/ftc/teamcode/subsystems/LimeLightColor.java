package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimeLightColor {

    private Limelight3A limelight;
    private Pose3D botpose;
    public double timeStamp; // in ns
    public Pose2d limePose;
    public double xOffset = 0; //X correction, in inch
    public double yOffset = 0; //Y correction, in inch
    public LimeLightColor(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // Pipeline 1 is for color
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
    }
    public void getLLPose(int cnt) {

        for (int i=0; i< cnt; i++) {
            LLResult result = limelight.getLatestResult();
            Log.v("LLCam-CLR", "i=" + i + " result=" + (result==null));
            if (result != null) {
                // Access general information
                botpose = result.getBotpose();
                timeStamp = System.nanoTime();
                if (result.isValid()) {
                    /*
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                     */
                    Log.v("LLCam-CLR", "tx=" + result.getTx()
                            + " ty=" + result.getTy());
                } else {
                    limePose = null;
                }
            }
        }
    }

    public double getTxTyTa(int cnt, int idx, double timeOutT) { //idx: 0: Tx, 1: Ty, 2: Ta
        double pre_meas=0, meas = 0, accum_meas = 0;
        int    realCnt = 0;
        LLResult result;
        boolean firstLp = true;
        double startTime = System.currentTimeMillis();
        boolean timeout = false;
        while (realCnt < cnt) {
            if (System.currentTimeMillis() - startTime > timeOutT*1000) {
                timeout = true;
                break;
            }
            result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                timeStamp = System.nanoTime();
                if (result.isValid()) {
                    switch (idx) {
                        case 0: meas = result.getTx(); break;
                        case 1: meas = result.getTy(); break;
                        case 2: meas = result.getTa(); break;
                    }
                    if (meas != pre_meas || firstLp) {
                        Log.v("LLCam-CLR", "idx=" + idx + " meas=" + meas);
                        accum_meas += meas;
                        realCnt += 1;
                    }
                    pre_meas = meas;
                    if (firstLp) firstLp = false;
                }
            }
        }
        Log.v("LLCam-CLR", "idx=" + idx
                +" accum_meas="+accum_meas+" cnt="+realCnt+" final:"+(accum_meas/realCnt));
        timeStamp = System.nanoTime() * 1e-9;
        if (timeout && firstLp) {
            return -1000;
        } else {
            return (accum_meas / realCnt);
        }
    }

    public void camOn(boolean camOn) {
        if (camOn)
            limelight.start();
        else
            limelight.stop();
    }
}
