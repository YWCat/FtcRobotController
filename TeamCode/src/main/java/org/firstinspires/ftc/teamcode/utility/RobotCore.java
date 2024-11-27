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
    public double mapJsComponents(double val, double radius, boolean slow){
        double factor = mapJsRadiusVal(radius,slow);
        if(radius<0.01){
            return 0;
        }
        return factor*val/radius;
    }

    public double mapJsRadiusVal(double jsVal, boolean slow){
        //https://www.desmos.com/calculator/ekyhsv03yo
        double startPos = 0.1; //a
        double startVal = 0.35; //b
        double endSlowPos = 0.95;//c
        double endSlowVal = 0.5; //d
        double maxVal = 1; //f
        double startSlope = startVal/startPos;
        double defSlope = (endSlowVal-startVal)/(endSlowPos-startPos);
        double endSlope = (maxVal-endSlowVal)/(1-endSlowPos);
        if(Math.abs(jsVal)<=startPos){
            return jsVal*startSlope;
        }
        else if(jsVal>startPos){
            double toReturn = (jsVal-startPos)*defSlope + startVal;
            if(!slow && jsVal>endSlowPos){
                toReturn = (jsVal-endSlowPos)*endSlope + endSlowVal;
            }
            return toReturn;
        }
        else{
            double toReturn = (jsVal+startPos)*defSlope - startVal;
            if(!slow && jsVal<-1*endSlowPos){
                toReturn = (jsVal+endSlowPos)*endSlope - endSlowVal;
            }
            return toReturn;
        }
    }


}
