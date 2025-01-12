package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    private RotatingSlide rotatingSlide;
    private static double savedSlideMotorLEncoder;
    private static double savedSlideMotorREncoder;
    private static final double TICKS_PER_REV = 537.7; //312 RPM
    private static final double PULLEY_DIAMETER_IN = (32 / 25.4);
    private double TARGET_TOLERANCE_INCH = 0.45;
    private final double TARGET_STATIC_THREASHOLD = 20; // in ticks??

    private final int MAX_VERTICAL_LIMIT_TICKS = 4400;     // Hard limit: 283mm * 3 converted to ticks: 4543. Also note slide cannot be fully retracted by 1cm.
    private final int MAX_HORIZONTAL_LIMIT_TICKS = 2090;   // to avoid exceed 42 inch
    private final double MAX_HORIZONTAL_LIMIT_IN = 15.38;
    public int effectiveMaxExtension = MAX_VERTICAL_LIMIT_TICKS;

    public static double MIN_POWER_UP_VERTICAL_HIGH = 0.5;
    public static double MIN_POWER_UP_VERTICAL = 0.15;
    public static double MIN_POWER_DOWN_VERTICAL = 0.0;


    public static double MAX_VELOCITY = TICKS_PER_REV * 312 / 60; //max ticks/s for RPM = 312, PPR (ticks / rev) = 537.7

    private boolean targetReached = true;
    private PIDFController pidfController;
    public static double kP = 0.75; //slow down at ~1 or 2 inches from target
    public static double kI = 0.0; //0.0000000001;
    public static double kD = 0.0;
    public static double PID_RANGE = 1.0;

    public static double minPowerUp = 0.0;
    public static double minPowerDown = 0.0;

    private boolean slideIsHorizontal = false;
    public static double HOLD_POWER_HANG = -0.3;

    private double powerL;
    private double powerR;

    private Telemetry telemetry;
    public SlideToPosition prevMoveSlideAction = null;
    public SetMotorPower prevSetMotorAction = null;

    //TODO: fine-tune LEVEL-HT values.
    public DualMotorSlide(RotatingSlide rotatingSlide){
        // Set up Left and Right motors
        this.rotatingSlide = rotatingSlide;
        RobotCore robotCore = RobotCore.getRobotCore();
        slideMotorL = robotCore.hardwareMap.get(DcMotorEx.class, RobotConfig.slideMotorL);
        slideMotorR = robotCore.hardwareMap.get(DcMotorEx.class, RobotConfig.slideMotorR);

        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */ //Do not reset encoder between runs

        PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients();
        coefficients.kP = kP;
        coefficients.kI = kI;
        coefficients.kD = kD;
        pidfController = new PIDFController(coefficients, 0.0, 0.0, 0.0 /*getHoldPower()*/);
        //Log.v("PIDLift: status: ", "init");
    }

    public void setTolerance (double tol){
        this.TARGET_TOLERANCE_INCH = tol;
    }

    public boolean getExceedsHorizontalLimit(double clearance){ //clearance is in inches
        int motorLPosition = slideMotorL.getCurrentPosition();
        int motorRPosition = slideMotorR.getCurrentPosition();
        double effectiveAngle =  rotatingSlide.getArmEffectiveAngle();
        if(effectiveAngle==0) {
            effectiveAngle = 0.00001;
        }
        double calculatedEffMaxExt= inchToTicks((MAX_HORIZONTAL_LIMIT_IN) / Math.cos((90- effectiveAngle)*Math.PI/180));
        boolean exceeds = (motorLPosition>= effectiveMaxExtension-clearance || motorRPosition>= effectiveMaxExtension-clearance);
        Log.i("horizontal limit slide extension", "slideL: " + motorLPosition + "slideR " + motorRPosition);
        Log.i("horizontal limit slide extension", "angle: " + effectiveAngle);
        Log.i("horizontal limit slide extension",  " limit1: " + effectiveMaxExtension + "limit2: "+calculatedEffMaxExt+ " exceeds: " + exceeds);
        return exceeds;
    }
    public boolean getExceedsHorizontalLimit(){
        return getExceedsHorizontalLimit(0);
    }
    public double mapPower(double power){
        //Log.i("mapPower", String.format("power %4.2f, minPowerUp %4.2f, minPowerDown %4.2f", power, minPowerUp, minPowerDown));
        if (Math.abs(power) < 10e-6){
            //Log.i("mapPower", "power almost 0");
            return getHoldPower();
        } else if (power > 0 && power <= minPowerUp) {
            //Log.i("mapPower", "else if 1");
            return minPowerUp;
        } else if (power < 0 && power >= minPowerDown) {
            //Log.i("mapPower", "else if 2");
            return minPowerDown;
        } else if (getExceedsHorizontalLimit()){
            if (power > 0) {
                Log.i("mapPower", "exceeds horizontal limit");
                return getHoldPower();
            }
        }
        //Log.i("mapPower", "return power no change");
        return power;
    }

    public void updatePowerWithEncoderDiff(boolean up){

        double factor = 1.0;
        int rightEncoder = slideMotorR.getCurrentPosition();
        int leftEncoder = slideMotorL.getCurrentPosition();
        if( leftEncoder!=0 && leftEncoder!=rightEncoder){
            Log.v("Slide Sync", "error exists: R="+rightEncoder + " L="+leftEncoder);
            double correction = (double)(leftEncoder - rightEncoder)*0.0076;
            if(up){
                factor = factor + correction;
            } else {
                factor = factor - correction;
            }
            /*
            * abs of (rightE - leftE) is around 20, with right being more when going up
            * and right less when going down
            * the og value is 800, but the left needs to be more affected
            * 20 / 800 is around 0.025
            * lets try 200?
            */
            if(factor>1.5){
                factor=1.5;
            } else if (factor<0.667){
                factor = 0.667;
            }
        }
        Log.v("power Sync", ""+factor);

        if (powerR * factor > 1.0) {
            powerL /= factor;
        } else {
            powerR *= factor;
        }
        Log.v("slide power Sync", "left power: "+powerL + "right power: " + powerR);
    }
    public void adjustLift(double velocityRatio){
        Log.i("Slide Power", "targetReached set to True at 1");
        targetReached = true;
        double power = velocityRatio;
        double velocity = MAX_VELOCITY * velocityRatio;
        if (getRightEncoder() < effectiveMaxExtension || velocityRatio < 0){
            slideMotorR.setPower(power);
            slideMotorL.setPower(power);
            //slideMotorL.setVelocity(velocity);
            //slideMotorR.setVelocity(velocity);
            Log.i("slide power", "adjust slide set power to " + power);
        } else {
            Log.i("slide power", "horizontal limit exceeded");
            holdPosition();
        }
    }

    public void forceSetPower(){
        slideMotorR.setPower(0);
        slideMotorL.setPower(0);
        Log.v("slide power", "force power to be 0");
    }

    public void holdPosition() {
        double power = getHoldPower();
        Log.v("Slide Power", String.format("Setting hold power: %4.2f", power));
        slideMotorL.setPower(power);
        slideMotorR.setPower(power);
    }

    private void updateTargetReached() {
        //if it has already reached a target, don't change it. only change when something has set it to not reached
        double motorRVel;
        double targetPos, currPos;

        motorRVel = Math.abs(slideMotorR.getVelocity());
        currPos = slideMotorR.getCurrentPosition();
        targetPos = inchToTicks(pidfController.targetPosition);

        this.targetReached = (this.targetReached || (motorRVel <= TARGET_STATIC_THREASHOLD && Math.abs(targetPos - currPos) <= inchToTicks(TARGET_TOLERANCE_INCH)));
        Log.v("Power sync target reached", this.targetReached+"");
        //telemetry.addData("slideMotorL.getVelocity() ", Math.abs(slideMotorL.getVelocity()));
        //telemetry.addData("lastError ", ticksToInches((int)Math.abs(targetPos - currPos)));
        //telemetry.addData("targetReached ", this.targetReached);
        //telemetry.update();
    }

    public boolean isLevelReached() {
        return this.targetReached;
    }

    public int inchToTicks(double inches) {
        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    } //1 inch is around 136 ticks (as of 12/02/24)

    public double ticksToInches(int ticks) {
        return ((double) ticks) / (TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    }

    public double getPosition() { //gets position in inches
        return slideMotorR.getCurrentPosition() / (TICKS_PER_REV/(PULLEY_DIAMETER_IN * Math.PI));
    }

    public int getLeftEncoder() {
        return slideMotorL.getCurrentPosition();
    }

    public double getRightEncoder() {
        return slideMotorR.getCurrentPosition();
    }

    public double getLeftVelocity() {
        return slideMotorL.getVelocity();
    }

    public double getRightVelocity() {
        return slideMotorR.getVelocity();
    }

    public void resetEncoder() {
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getEffExpansionInches(){
        return ticksToInches(effectiveMaxExtension);
    }

    public double getMaxHorizontalLimitInches(){
        return MAX_HORIZONTAL_LIMIT_IN;
    }


    public final class SlideToPosition implements Action {
        private boolean cancelled;
        private double powerCap = 1;
        private long startTime;
        private int targetPosition; //ticks
        private boolean runStarted;
        private long timeout = 10000;

        public SlideToPosition(int pos, double powerCap) {
            changeTarget(pos, powerCap);
            Log.i(" Slide Actions", "Created new action");
        }

        public void changeTarget(int pos, double powerCap) {
            Log.i("Slide Actions", "move slide action modified, target pos" + pos);
            targetPosition = pos;
            runStarted = false;
            cancelled = false;
            this.powerCap = powerCap;
            if(this.powerCap > 0){
                this.powerCap = Math.min(this.powerCap, PID_RANGE);
            } else {
                this.powerCap = Math.max(this.powerCap, -1 * PID_RANGE);
            }
        }

        public boolean run(@NonNull TelemetryPacket p) {

            if (!runStarted) {
                pidfController.setOutputBounds(-1.0 * this.powerCap, 1.0 * this.powerCap);
                pidfController.reset();
                pidfController.targetPosition = ticksToInches(targetPosition);
                Log.v("Slide Action", "target pos ticks = " + targetPosition + " inches = "+ ticksToInches(targetPosition));

                targetReached = false;
                startTime = System.currentTimeMillis();
                runStarted = true;
                return true;
            } else {
                updateTargetReached();
                //Log.i("slideMotor RobotActions", "motor pos: " + slideMotorL.getCurrentPosition() + "target pos: " + slideMotorL.getTargetPosition());
                if(!isLevelReached() && (System.currentTimeMillis() - startTime < timeout) && !cancelled){
                    double measuredPositionL = ticksToInches(slideMotorL.getCurrentPosition());
                    double measuredPositionR = ticksToInches(slideMotorR.getCurrentPosition());
                    double powerFromPIDF = pidfController.update(measuredPositionR);
                    //if you want to add a constant upward power pls adjust kS instead
                    Log.v("Slide Extension & Power Sync", String.format("Target pos: %4.2f, current left pos: %4.2f, current right pos: %4.2f, last error: %4.2f, velocity: %4.2f, set power to: %4.2f", pidfController.targetPosition, measuredPositionL, measuredPositionR, pidfController.lastError, slideMotorL.getVelocity(), powerFromPIDF));
                    Log.v("Slide Extension", "Max extension: " + ticksToInches(effectiveMaxExtension));
                    powerFromPIDF = powerFromPIDF + getHoldPower();
                    Log.v("power sync 2", "after adding hold power" + powerFromPIDF);
                    double mappedPower = mapPower(powerFromPIDF);
                    powerL = mappedPower;
                    powerR = mappedPower;
                    updatePowerWithEncoderDiff(mappedPower>0);
                    Log.v("Power Sync power calcs", String.format("PID C power: %4.2f, Min/Max map power: %4.2f", powerFromPIDF, mappedPower));
                    slideMotorL.setPower(powerL);
                    slideMotorR.setPower(powerR);
                    Log.v("Slide Power Sync", "Action Should set power: left: " + powerL + "right: " + powerR + ". Actual power: Left: " + slideMotorL.getPower() + " Right: " + slideMotorR.getPower());
                    return true;

                } else {

                    holdPosition();
                    if (cancelled){
                        targetReached = true;
                        Log.v("Slide Action", "cancelled");
                    }
                    if(isLevelReached()){
                        Log.v("Slide Action", "target reached");
                    }
                    if(System.currentTimeMillis() - startTime >= timeout) {
                        targetReached = true;
                        Log.v("Slide Action", "timeout");
                    }
                    Log.i("AUTO", "Done slide: target = "+ targetPosition);
                    return false;
                }
            }
        }

        public void cancel(){
            Log.v("Slide Power", "invoking holdPosition at 2");
            cancelled = true;
        }
    }

    public class SetMotorPower implements Action {
        private double power = 0;

        public SetMotorPower(double power) {
            changeTarget(power);
            Log.i(" Slidemotor RobotActions", "Created new action setMotorPower");
        }

        public void changeTarget( double power) {
            Log.i("slideMotor RobotActions", "setMotorPower action modified, power = " + power);
            this.power = power;
        }

        public boolean run(@NonNull TelemetryPacket packet){
            targetReached = false; //slide not in transition state
            slideMotorL.setPower(power);
            slideMotorR.setPower(power);
            Log.i("Slide Power", String.format("setMotorPower Action set flat motor power: %4.2f ", power));
            return false;
        }

    }

    public SlideToPosition getSlideToPosition(int posTicks, double powerCap, boolean forceNew){
        if(!forceNew){
            if(prevMoveSlideAction == null){
                prevMoveSlideAction = new SlideToPosition(posTicks, powerCap);
            } else {
                prevMoveSlideAction.changeTarget(posTicks, powerCap);
            }
            return prevMoveSlideAction;
        } else{
            return new SlideToPosition(posTicks, powerCap);
        }
    }

    public SlideToPosition getSlideToPosition(double posInches,  double powerCap, boolean forceNew){
        return getSlideToPosition(inchToTicks(posInches), powerCap, forceNew);
    }

    public SetMotorPower getSetMotorPower(double power, boolean forceNew) {
        if (!forceNew) {
            if (prevSetMotorAction == null) {
                prevSetMotorAction = new SetMotorPower(power);
            } else {
                prevSetMotorAction.changeTarget(power);
            }
            return prevSetMotorAction;
        } else {
            return new SetMotorPower(power);
        }
    }

    public void changeHorizontalSetting() {
        double effectiveAngle = Math.abs(rotatingSlide.getArmEffectiveAngle());
        if(effectiveAngle == 0){ //avoid divide by zero
            effectiveAngle = 0.001;
        }
        effectiveMaxExtension = inchToTicks(MAX_HORIZONTAL_LIMIT_IN / Math.cos((90- effectiveAngle)*Math.PI/180)); //math.cos uses radians so convert here
        if(effectiveMaxExtension > MAX_VERTICAL_LIMIT_TICKS) {
            effectiveMaxExtension = MAX_VERTICAL_LIMIT_TICKS;
        }
        //Log.i("max extension under horizontal limit", "angle: " + effectiveAngle + "max ex:" + ticksToInches(effectiveMaxExtension)) ;
        slideIsHorizontal = effectiveAngle > rotatingSlide.getHorizontalThresholdAngle();
        if (slideIsHorizontal) {
            minPowerUp = 0;
            minPowerDown = 0;
            //Log.i("HorizontalLimit 1", "from dualMotorSlide, is horizontal");
        } else {
            //Log.i("HorizontalLimit 1", "from dualMotorSlide, is NOT horizontal");
            if (getPosition() < 26) {
                minPowerUp = MIN_POWER_UP_VERTICAL;
            } else {
                minPowerUp = MIN_POWER_UP_VERTICAL_HIGH;
            }
            minPowerDown = MIN_POWER_DOWN_VERTICAL;
        }
    }

    //FOR DEBUG CONSTANTLY PRINTING MOTOR POWER
    public String printPowerAndVelocity(){
        return "leftP: " + slideMotorL.getPower() + " rightP:" + slideMotorR.getPower() + " leftV:" + slideMotorL.getVelocity() + " rightV:" + slideMotorR.getVelocity();
    }

    public double getHoldPower() {
        if (slideIsHorizontal) {
            Log.i("holdPower", "0");
            return 0;
        } else {
            if (getPosition() > 26) {
                Log.i("holdPower", "tall mode, 0.25");
                return 0.25;
            } else {
                Log.i("holdPower", "normal mode " + 0.1);
                return 0.1;
            }
        }
    }
}
