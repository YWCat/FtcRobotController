package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotCore {
    public static HardwareMap hardwareMap = null;
    public static Telemetry telemetry = null;
    private static RobotCore robotCoreInstance = null;

    public RobotCore(OpMode opMode){

        hardwareMap = opMode.hardwareMap;
        robotCoreInstance = this;
    }
    public static RobotCore getRobotCore() throws RuntimeException{
        if(robotCoreInstance==null){
            throw new RuntimeException("RobotCore must be initialized first");
        } else{
            return robotCoreInstance;
        }
    }


}
