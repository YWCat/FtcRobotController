package org.firstinspires.ftc.teamcode.utility;

import android.util.Log;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.lang.reflect.Method;
import java.util.List;


public class LoopUpdater {

    private FtcDashboard dashboard;
    private List<LynxModule> allHubs;
    private ArrayList<Action> activeActions = new ArrayList<Action>();
    private ArrayList<PeriodicUpdatingEntity> updatingEntities = new ArrayList<PeriodicUpdatingEntity>();
    public static LoopUpdater LoopUpdaterInstance = null;

    public LoopUpdater() {

        dashboard = FtcDashboard.getInstance();
        LoopUpdaterInstance = this;

        // Enable bulk read (auto bulk caching mode) to optimize loop intervals
        // Note: set bulk cashing mode to MANUAL requires clearing cache in each loop, thus this is done in this class.
        // Note: all motors need to be accessed through DCMotorEx in order to take advantage of bulk read
        allHubs = RobotCore.getRobotCore().hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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
    public void clearActions(){
        TelemetryPacket packet = new TelemetryPacket();
        for(Action action:activeActions){
            // Method name to check
            String methodName = "cancel";
            // Get the class of the object
            Class<?> clazz = action.getClass();
            // If the class is a SequentialAction or a Parallel Action, access its contents
            List<Action> actionsToCancel = new ArrayList<Action>();
            if (clazz == SequentialAction.class){
                actionsToCancel = ((SequentialAction) action).getInitialActions();
            } else if (clazz == ParallelAction.class){
                actionsToCancel = ((ParallelAction) action).getInitialActions();
            } else{
                actionsToCancel.add(action);
            }

            Log.i("robotActions cancelled", "class to cancel" + clazz);

            for (Action actionToCancel : actionsToCancel){
                Method method = null;
                try {
                    // Check if the method exists
                    method = actionToCancel.getClass().getMethod(methodName);

                } catch (NoSuchMethodException e) {
                    Log.e("robotActions cancelled", "Method not found: " + methodName);
                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(method != null){
                    try {
                        method.invoke(actionToCancel);
                        Log.i("robotActions cancelled", "Method found! " + methodName);
                    } catch (IllegalAccessException e) {
                        Log.e("robotActions cancelled","Cannot access the method: " + methodName);
                        e.printStackTrace();
                    } catch (java.lang.reflect.InvocationTargetException e) {
                        Log.e("robotActions cancelled","Error while invoking the method: " + methodName);
                        Throwable cause = e.getCause();
                        if (cause != null) {
                            Log.e("robotActions cancelled","Cause: " + cause.getMessage());
                            cause.printStackTrace();
                        }
                    }
                }
            }

        }
        activeActions.clear();
        Log.i("UpdateActs RobotActions", "Actions Active: " + activeActions.size());
    }
    public ArrayList<Action> getActiveActions(){
        return activeActions;
    }
    public void updateAndRunAll(){

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        TelemetryPacket packet = new TelemetryPacket();

        for (PeriodicUpdatingEntity entity:updatingEntities){
            entity.update(packet);
        }

        ArrayList<Action> unfinishedActions = new ArrayList<Action>();
        for(Action action:activeActions){
            boolean needsContinue = action.run(packet);
            if (needsContinue){
                unfinishedActions.add(action);
            }
        }
        activeActions = unfinishedActions;
        Log.i("UpdateActs", "Actions Active: " + activeActions.size());

        dashboard.sendTelemetryPacket(packet);
    }
}
