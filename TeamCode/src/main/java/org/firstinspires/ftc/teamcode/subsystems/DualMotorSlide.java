package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.PIDFController;
import org.firstinspires.ftc.teamcode.utility.RobotCore;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

@Config
//synchronizes lifts, provides commands to move to a position
public class DualMotorSlide {
    //Hardware: 2 lift motors
    public DcMotorEx slideMotorL;
    public DcMotorEx slideMotorR;
    private static final double TICKS_PER_REV = 537.7; //5203-2402-0027, 223 RPM
    private static final double PULLEY_DIAMETER_IN = (32 / 24.5); //3407-0002-0112 // = 1.269685 inches
    private final int TARGET_TOLERANCE_TICKS = (int) (0.3*TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    private Telemetry telemetry;
    private boolean targetReached = true;
    public final double RIGHT_TO_LEFT_SYNC = 1;
    private final int MAX_HT_TICKS = 3700;
    private final int MIN_HT_TICKS = 0;//(int) (0TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI)); //dont get any lower than
    private final double FAST_POWER = 0.6;
    private final double SLOW_POWER = 0.3;
    private VoltageSensor batteryVoltageSensor;

    public enum Mode {
        BOTH_MOTORS_PID,
        RIGHT_FOLLOW_LEFT
    };
    public Mode mode;
    // Public just to allow tuning through Dashboard
    public static double  UP_VELOCITY = 2796; //max ticks/s for RPM = 312, PPR (ticks / rev) = 537.7
    public static double[] LEVEL_HT = {13, 7, 17.0, 29.0}; // in inches, please fine-tune
    public static double[] LEVEL_HT_AUTO = {7, 4, 17.0, 29.0};
    //4 levels: 0 = minimum arm swing height; 1= ground, 2= low, 3= middle, 4= high,
    //0:5.0

    private PIDFController pidfController;
    public static double kP = 0.25; //slow down at 2 inches from target
    public static double kI = 0.0; //0.0000000001;
    public static double kD = 0.0;
    public static double kA = 0.0;
    public static double kV = 0.0;
    public static double kS = 0.002;
    public static double PID_RANGE = 1;
    public static double MIN_HOLD_POWER_UP = 0.002;
    public static double MIN_HOLD_POWER_UP_VERTICAL = 0.002;
    public static double MIN_HOLD_POWER_UP_HORIZONTAL = 0;
    public static double MIN_HOLD_POWER_DOWN = -0.04;
    private double powerFromPIDF;
    public static slideToPosition prevMoveSlideAction = null;


    //TODO: fine-tune LEVEL-HT values.
    public DualMotorSlide(){
        // Set up Left and Right motors
        RobotCore robotCore = RobotCore.getRobotCore();
        slideMotorL = robotCore.hardwareMap.get(DcMotorEx.class, RobotConfig.slideMotorL);
        slideMotorR = robotCore.hardwareMap.get(DcMotorEx.class, RobotConfig.slideMotorR);
        slideMotorL.setTargetPositionTolerance(inchToTicks(0.3)); //TARGET_TOLERANCE_TICKS);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorL.setTargetPositionTolerance(TARGET_TOLERANCE_TICKS);
        slideMotorR.setTargetPositionTolerance(TARGET_TOLERANCE_TICKS);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        //slideMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        if (mode== Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorR.setTargetPosition(0);
            slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients();
            coefficients.kP = kP;
            coefficients.kI = kI;
            coefficients.kD = kD;
            pidfController = new PIDFController(coefficients, kV, kA, kS);
            pidfController.setOutputBounds(-1.0*PID_RANGE, 1.0*PID_RANGE);
            pidfController.reset();
        }
        //Log.v("PIDLift: status: ", "init");
    }

    public double mapPower(double power){
        if(Math.abs(power) < 10e-6){
            return 0;
        } else if (power > 0 && power <= MIN_HOLD_POWER_UP) {
            return MIN_HOLD_POWER_UP;
        } else if (power < 0 && power >= MIN_HOLD_POWER_DOWN) {
            return MIN_HOLD_POWER_DOWN ;
        }
        if(slideMotorL.getCurrentPosition()>=MAX_HT_TICKS || slideMotorR.getCurrentPosition()>=MAX_HT_TICKS){
            if (power > 0 ) {
                return MIN_HOLD_POWER_UP;
            } else if (power < 0 ) {
                return Math.min(MIN_HOLD_POWER_DOWN, power);//MIN_HOLD_POWER_DOWN ;
            }
        }
        return power;
    }
    public double getLeftFactor(){
        double factor = 1.0;
        int rightEncoder = slideMotorR.getCurrentPosition();
        int leftEncoder = slideMotorL.getCurrentPosition();
        if( rightEncoder!=0 && leftEncoder!=rightEncoder){
            Log.v("SlideSync", "error exists: R="+rightEncoder + " L="+leftEncoder);
            //factor = 1.0 + Math.abs((double)(rightEncoder - leftEncoder))/800;
            /*
            * abs of (rightE - leftE) is around 20, with right being more when going up
            * and right less when going down
            * the og value is 800, but the left needs to be more affected
            * 20 / 800 is around 0.025
            * lets try 200?
            */
            if(factor>1.5){
                factor=1.5;
            } else if (factor<0.5){
                factor = 0.5;
            }
        }
        Log.v("SlideSync", ""+factor);
        return factor;

    }
    public void goToLevel(int level){
        //4 levels: 0 ground, 1 low, 2 middle, 3 high, 4 (minimum height for free chain bar movement)
        int targetPosition = inchToTicks(LEVEL_HT[level]);
        goToHt(targetPosition);
        //Log.v("ChainBar", "going to level" + level);

    }
    //for going to non-junction heights
    public void goToHt(int ticks) {
        ticks = Math.min(MAX_HT_TICKS, ticks);
        targetReached=false;
        if(mode==Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotorR.setTargetPosition(ticks);
            slideMotorR.setVelocity(UP_VELOCITY);
            //fine tune velocity?
            //In case if i should set right motor right when i set left motor (prob not useful)
            slideMotorL.setVelocity(UP_VELOCITY*RIGHT_TO_LEFT_SYNC);
        }
        else {
            pidfController.reset();
            pidfController.targetPosition = (ticksToInches(ticks));
        }
        //Log.v("PIDLift: Debug: ", String.format("Lift moving to height %f, error is %f4.2, power output = %f4.2", ticksToInches(slideMotorL.getTargetPosition()), pidfController.lastError,powerFromPIDF));
    }

    public void goToHtInches(double inches) {
        goToHt(inchToTicks(inches));
    }

    public void goToRelativeOffset(double inches) {
        targetReached=false;
        int currPosTicks = slideMotorR.getCurrentPosition();
        int targetPosTicks = currPosTicks + inchToTicks(inches);

        //Log.v("AUTOCMD DEBUG", "currPosTicks: " + currPosTicks);
        //Log.v("AUTOCMD DEBUG", "offset Inches: " + inches);
        //Log.v("AUTOCMD DEBUG", "targetPosTicks: " + targetPosTicks);
        this.goToHt(targetPosTicks);
    }

    public void applyStaticOffset(int direction, double power) {
        if(mode == Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotorR.setPower(power*direction);
        }
        else{
            slideMotorR.setPower(mapPower(power*direction));
            slideMotorL.setPower(mapPower(power*direction) * getLeftFactor());
            Log.v("SlideSync", "R power=" +mapPower(power*direction));
            Log.v("SlideSync", "L power=" +mapPower(power*direction)* getLeftFactor());
        }
        //Log.v("PIDLift: status: ", "applyStaticOffset");
    }

    public void adjustLift(double direction, boolean slow){
        targetReached=true;
        double power = SLOW_POWER;
        if(!slow){
            power = FAST_POWER;
        }
        if(mode == Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotorR.setPower(power*direction);
        }
        else{
            slideMotorL.setVelocity(UP_VELOCITY * direction);
            slideMotorR.setVelocity(UP_VELOCITY * direction); //use velocity instead of power?
            powerFromPIDF = power * direction;
            powerFromPIDF = mapPower(powerFromPIDF);
            //slideMotorR.setPower(powerFromPIDF);
            //slideMotorL.setPower(powerFromPIDF*getLeftFactor());

        }
    }

    public void stopMotor(){
        if(mode==Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorL.setVelocity(0.0);
            slideMotorR.setVelocity(0.0);
        }
        //slideMotorL.setVelocity(0.0);
        //slideMotorR.setVelocity(0.0);
        slideMotorL.setPower(MIN_HOLD_POWER_UP);
        slideMotorR.setPower(MIN_HOLD_POWER_UP);
    }

    private void updateTargetReached() {
        //if it has already reached a target, don't change it. only change when something has set it to not reached
        double motorLVel;
        double targetPos, currPos;

        motorLVel = Math.abs(slideMotorL.getVelocity());
        currPos = slideMotorL.getCurrentPosition();

        if (mode == Mode.RIGHT_FOLLOW_LEFT) {
            targetPos = slideMotorR.getTargetPosition();
        } else {
            targetPos = inchToTicks(pidfController.targetPosition);
        }
        this.targetReached = (this.targetReached || (motorLVel <= 20 && Math.abs(targetPos - currPos) <= (TARGET_TOLERANCE_TICKS)));
        //telemetry.addData("slideMotorL.getVelocity() ", Math.abs(slideMotorL.getVelocity()));
        //telemetry.addData("lastError ", ticksToInches((int)Math.abs(targetPos - currPos)));
        //telemetry.addData("targetReached ", this.targetReached);
        //telemetry.update();
    }

    public boolean isLevelReached(){
        return this.targetReached;
    }

    public int inchToTicks(double inches) {
        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    }
    public double ticksToInches(int ticks){
        return ((double) ticks) / (TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    }

    public double getPosition(){
        return slideMotorR.getCurrentPosition() / (TICKS_PER_REV/(PULLEY_DIAMETER_IN * Math.PI));
    }

    public int getLeftEncoder(){
        return slideMotorL.getCurrentPosition();
    }
    public double getRightEncoder(){
        return slideMotorR.getCurrentPosition();
    }

    public double getLeftVelocity(){
        return slideMotorL.getVelocity();
    }public double getRightVelocity(){
        return slideMotorR.getVelocity();
    }

    public void resetEncoder(){
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getTargetPos(){
        return slideMotorR.getTargetPosition();
    }

    public double getPIDPower(){
        return powerFromPIDF;
    }

    public final class slideToPosition implements Action {
        private boolean cancelled;
        private long startTime;
        private int targetPosition; //ticks
        private boolean runStarted;
        private long timeout = 10000;

        public slideToPosition(int pos) {
            changeTarget(pos);
            Log.i(" Slidemotor RobotActions", "Created new action");
        }

        public void changeTarget(int pos) {
            Log.i("slideMotor RobotActions", "move slide action modified, target pos" + pos);
            targetPosition = pos;
            runStarted = false;
            targetReached = false;
            cancelled = false;
        }

        public boolean run(@NonNull TelemetryPacket p) {
            if (!runStarted) {
                if (mode == Mode.RIGHT_FOLLOW_LEFT) {
                    slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotorR.setTargetPosition(targetPosition);
                    slideMotorR.setVelocity(UP_VELOCITY);
                    //fine tune velocity?
                    //In case if i should set right motor right when i set left motor (prob not useful)
                    slideMotorL.setVelocity(UP_VELOCITY * RIGHT_TO_LEFT_SYNC);
                } else {
                    pidfController.reset();
                    pidfController.targetPosition = (ticksToInches(targetPosition));
                    Log.v("slidemotor slidetargetPosition", "target pos ticks = " + targetPosition + " inches = "+ ticksToInches(targetPosition));
                    targetReached = false;
                }

                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            } else {
                if (mode == Mode.RIGHT_FOLLOW_LEFT) {
                    if (slideMotorL.isBusy() && System.currentTimeMillis() - startTime < timeout && !cancelled) {
                        Log.i("slideMotor RobotActions", "motor pos: " + slideMotorL.getCurrentPosition() + "target pos: " + slideMotorL.getTargetPosition());
                        return true;
                    } else {
                        if (!slideMotorL.isBusy()) {
                            Log.i("slideMotor RobotActions", "slide done moving: " + slideMotorL.getCurrentPosition() + "target pos: " + slideMotorL.getTargetPosition());
                        } else {
                            Log.i("slideMotor RobotActions", "timeout; position: " + slideMotorL.getCurrentPosition() + "target pos: " + slideMotorL.getTargetPosition());
                        }
                        slideMotorL.setVelocity(0.0);
                        slideMotorR.setVelocity(0.0);
                        slideMotorL.setPower(MIN_HOLD_POWER_UP);
                        slideMotorR.setPower(MIN_HOLD_POWER_UP);
                        return false;
                    }
                } else {
                    updateTargetReached();
                    Log.i("slideMotor RobotActions", "motor pos: " + slideMotorL.getCurrentPosition() + "target pos: " + slideMotorL.getTargetPosition());
                    if(!isLevelReached() && System.currentTimeMillis() - startTime < timeout && !cancelled){
                        double measuredPositionL = ticksToInches(slideMotorL.getCurrentPosition());
                        double measuredPositionR = ticksToInches(slideMotorR.getCurrentPosition());
                        powerFromPIDF = pidfController.update(measuredPositionL);
                        //if you want to add a constant upward power pls adjust kS instead
                        Log.v("PIDLift", String.format("Target pos: %4.2f, current left pos: %4.2f, current right pos: %4.2f, last error: %4.2f, velocity: %4.2f, set power to: %4.2f",
                                pidfController.targetPosition, measuredPositionL, measuredPositionR, pidfController.lastError, slideMotorL.getVelocity(), powerFromPIDF));
                        //telemetry.addData("Target pos", pidfController.getTargetPosition());
                        //telemetry.addData("Measure pos", measuredPosition);
                        //telemetry.addData("slidePower", powerFromPIDF);
                        //telemetry.update();
                        powerFromPIDF = mapPower(powerFromPIDF);
                        slideMotorL.setPower(powerFromPIDF*getLeftFactor());
                        slideMotorR.setPower(powerFromPIDF);
                        Log.v("PIDLift", "Actual power: left: " + slideMotorL.getPower() + "right: " + slideMotorR.getPower());
                        return true;
                    } else{

                        if(isLevelReached()){
                            Log.v("slideMotorStop RobotActions", "target reached");
                        }
                        if(System.currentTimeMillis() - startTime >= timeout) {
                            Log.v("slideMotorStop RobotActions", "timeout");
                        } if (cancelled){
                            Log.v("slideMotorStop RobotActions", "cancelled");
                        }
                    }
                    return false;
                }

            }
        }
        public void cancel(){
            slideMotorL.setVelocity(0.0);
            slideMotorR.setVelocity(0.0);
            slideMotorL.setPower(MIN_HOLD_POWER_UP);
            slideMotorR.setPower(MIN_HOLD_POWER_UP);
            cancelled = true;
        }
    }
    public slideToPosition getSlideToPosition(int pos, boolean forceNew){
        if(!forceNew){
            if(prevMoveSlideAction == null){
                prevMoveSlideAction = new slideToPosition(pos);
            } else {
                prevMoveSlideAction.changeTarget(pos);
            }
            return prevMoveSlideAction;
        } else{
            return new slideToPosition(pos);
        }
    }
    public void changeHoldPower(boolean horizontal){
        MIN_HOLD_POWER_UP = MIN_HOLD_POWER_UP_VERTICAL;
        if(horizontal) {
            MIN_HOLD_POWER_UP = MIN_HOLD_POWER_UP_HORIZONTAL;
        }
    }


}
