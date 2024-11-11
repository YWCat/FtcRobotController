package org.firstinspires.ftc.teamcode.utility;

import android.util.Log;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class LoopUpdater {
    private FtcDashboard dashboard;
    private SmartGamepad smartGamepad;
    private ArrayList<Action> activeActions = new ArrayList<Action>();
    private ArrayList<PeriodicUpdatingEntity> updatingEntities = new ArrayList<PeriodicUpdatingEntity>();
    public static LoopUpdater LoopUpdaterInstance = null;
    public LoopUpdater(){
        dashboard = FtcDashboard.getInstance();
    }
    public static LoopUpdater getSharedLoopUpdater() throws RuntimeException{
        if(LoopUpdaterInstance==null){
            throw new RuntimeException("LoopUpdater must be initialized first");
        } else{
            return LoopUpdaterInstance;
        }
    }
    public void addPeriodicUpdatingEntity(PeriodicUpdatingEntity entity){
        updatingEntities.add(entity);
    }
    public void addAction(Action action){
        if(!activeActions.contains(action)){
            activeActions.add(action);
        }
    }
    public void updateAndRunAll(){
        TelemetryPacket packet = new TelemetryPacket();

        for (PeriodicUpdatingEntity entity:updatingEntities){
            entity.update(packet);
        }

        ArrayList<Action> unfinishedActions = new ArrayList<Action>();
        for(Action action:activeActions){
            boolean needsContinue = action.run(packet);
            if (needsContinue){
                unfinishedActions.add(action);
            } else {
                Log.i("UpdateActions", "Actions Active: " + unfinishedActions.size());
            }
        }
        activeActions = unfinishedActions;
        Log.i("UpdateActs", "Actions Active: " + activeActions.size());

        dashboard.sendTelemetryPacket(packet);
    }
}
