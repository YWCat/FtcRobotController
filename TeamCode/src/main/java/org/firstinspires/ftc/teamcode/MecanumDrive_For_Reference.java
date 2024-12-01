package org.firstinspires.ftc.teamcode;

/*  //UNCOMMENT HERE
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.other.Utilities;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.Math;
import java.sql.Array;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import android.util.Log;

@Config
public final class MecanumDrive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive model parameters
        public double inPerTick = Robot.inPerTick;
        public double lateralInPerTick = 0.002589030501021076;

        // By measurement, the track width is 12.0/9.25 track width. With AngularRampLogger I got 3347 as track with.
        // But this does not make the robot run the right angle. By create a TurnAction (360 degree) and ensure the
        // robot actually turns 360 degree, I got track width 2925.
        public double trackWidthTicks = 2925;

        // feedforward parameters (in tick units)
        public double kS = 0.8069260422250029;
        public double kV = 0.0006006719748419291;
        public double kA = 0.0002;

        // path profile parameters (in inches)
        public double maxWheelVel = 1.5 * 50;
        public double minProfileAccel = 1.5 * -50;
        public double maxProfileAccel = 1.5 * 50;

        // turn profile parameters (in radians)
        public double maxAngVel = 1.5 * Math.PI; // shared with path
        public double maxAngAccel = 1.5 * Math.PI;

        // path controller gains
        public double axialGain = 3.0; // 3.0;
        public double lateralGain = 2.0; // 3.0

        // Having a big headingGain (for example, 10), can cause robot to keep adjust heading without moving forward
        // It seems a relative small headingGain makes robot drive (toward target) more smoothly
        public double headingGain = 1.5;
        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // 1.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final AccelConstraint transitionAccelConstraint =
            new ProfileAccelConstraint(-5, 10);

    public final AccelConstraint tuningnAccelConstraint =
            new ProfileAccelConstraint(-3, 3);

    public final VelConstraint reachingVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(20),
                    new AngularVelConstraint(Math.PI / 2)
            ));

    public final VelConstraint tuningVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(3),
                    new AngularVelConstraint(Math.PI / 8)
            ));


    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    public Pose2d pose;

    private class Pose2dWithTime {
        public Pose2d pose;
        public long nanoTime;
        Pose2dWithTime(Pose2d pose, long nanoTime) {
            this.pose = pose;
            this.nanoTime = nanoTime;
        }
    }
    private final LinkedList<Pose2dWithTime> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public NearTargetAdjustment nearTargetCorrection = null;
    public TrajectoryAccuracy trajectoryAccuracy = null;

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront  = hardwareMap.get(DcMotorEx.class, Robot.MOTOR_NAME_LF);
        leftBack   = hardwareMap.get(DcMotorEx.class, Robot.MOTOR_NAME_LB);
        rightBack  = hardwareMap.get(DcMotorEx.class, Robot.MOTOR_NAME_RB);
        rightFront = hardwareMap.get(DcMotorEx.class, Robot.MOTOR_NAME_RF);

        leftFront.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(  DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftFront.setDirection( DcMotor.Direction.REVERSE);
        leftBack.setDirection(  DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection( DcMotor.Direction.FORWARD);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);

        initDashboardGraph();
    }

    public void setAdjustedMotorPower(DcMotorEx motor, double power, double ratio, double min_power) {

        if (!Robot.dryRun) {
            double power_to_use = power;
            double powerAbs = Math.abs(power);
            if (powerAbs < Math.abs(min_power) && powerAbs > 1e-2) {
                power_to_use = min_power * (power / powerAbs);
            }
            power_to_use *= ratio;
            motor.setPower(power_to_use);
        }
    }

    // Passing 1 to "new MecanumKinematics()" looks werid at the first glance. It is probably becuase
    // this function is only for (human) driver control.
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        setAdjustedMotorPower(leftFront,  wheelVels.leftFront.get(0)  / maxPowerMag, Robot.leftFrontRatio,  Robot.driveMotorPowerMin);
        setAdjustedMotorPower(leftBack,   wheelVels.leftBack.get(0)   / maxPowerMag, Robot.leftBackRatio,   Robot.driveMotorPowerMin);
        setAdjustedMotorPower(rightBack,  wheelVels.rightBack.get(0)  / maxPowerMag, Robot.rightBackRatio,  Robot.driveMotorPowerMin);
        setAdjustedMotorPower(rightFront, wheelVels.rightFront.get(0) / maxPowerMag, Robot.rightFrontRatio, Robot.driveMotorPowerMin);
    }

    public void initDashboardGraph() {

        TelemetryPacket p = new TelemetryPacket();
        p.put("x", 0);
        p.put("y", 0);
        p.put("heading (deg)", 0);
        p.put("xError", 0);
        p.put("yError", 0);
        p.put("headingError (deg)", 0);

        p.put("x_vel", 0);
        p.put("y_vel", 0);
        p.put("head_vel", 0);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(p);
    }

    public final class NearTargetAdjustment {
        private final Vector2d nearTargetEntry;
        private final double nearTargetRange;
        private final Pose2d targetPos;
        private boolean enteredNearRegion = false;


        public NearTargetAdjustment(Vector2d nearTargetEntry, double nearTargetRange, Pose2d targetPos) {

            this.nearTargetEntry = nearTargetEntry;
            this.nearTargetRange = nearTargetRange;
            this.targetPos = targetPos;
        }

        private boolean hasEnteredNearRegion(Pose2d current) {

            if (!enteredNearRegion) {
                Log.i("move to target", String.format("Is robot near target? Current: %4.2f, %4.2f, %4.2f, expected: %4.2f, %4.2f, toleranc: %4.2f, timestamp: %d",
                        current.position.x, current.position.y, current.heading.toDouble(), nearTargetEntry.x, nearTargetEntry.y, nearTargetRange, System.nanoTime()));

                double xDiff = current.position.x - nearTargetEntry.x;
                double yDiff = current.position.y - nearTargetEntry.y;
                if (Math.sqrt((xDiff * xDiff + yDiff * yDiff)) < nearTargetRange) {
                    enteredNearRegion = true;
                }
            }
            return enteredNearRegion;
        }

        /*
        If robot hasn't entered the area that is close to target (defined by nearTargetEntry and nearTargetRange), return null
        If robot is near target (or has entered the region) but cannot see target, return null
        If robot is near target (or has entered the region), sees the target, and haven't reached target, return the adjusted position
        If robot reached target (error within tolerance), return null
         */
/*  //UNCOMMENT HERE
        public Pose2d adjustedPosUsingTarget(Pose2d current) {

            if (!hasEnteredNearRegion(current)) {
                return null;
            }

            Log.i("move to target", String.format("Robot is near target at: %4.2f, %4.2f, %4.2f, timestamp %d",
                    current.position.x, current.position.y, current.heading.toDouble(), System.nanoTime()));

            AprilTagDetection detection = Vision.lookForAprilTag(4, true);
            if (detection != null) {
                // ftcPose: x: offset of left/right (left being negative), y: distance in the front,
                //          yaw being angle rotated left/right (facing left being negative
                // Example ftcPose get: x: -1.17, y: 27.80, pitch: -7.96

                Utilities utilities = Utilities.getSharedUtility();
                // poseTargetRelative is position of the target, in robot centric coordination system
                Vector2d poseTargetRelative = new Vector2d(detection.ftcPose.y, -detection.ftcPose.x);

                // convert to field centric coordination. But instead of using robot heading, use yaw, because
                // we trust the predefined target position more than the robot localization.
                double speculatedHeading = utilities.normalizeRadian(targetPos.heading.toDouble() - Math.toRadians(detection.ftcPose.yaw));
                double rotatedX = poseTargetRelative.x * Math.cos(speculatedHeading) - poseTargetRelative.y * Math.sin(speculatedHeading);
                double rotatedY = poseTargetRelative.y * Math.cos(speculatedHeading) + poseTargetRelative.x * Math.sin(speculatedHeading);

                Log.i("move to target", String.format("April tag position rotated to field coordination, using speculated heading %4.2f: %4.2f, %4.2f, expected positon: %4.2f, %4.2f",
                        speculatedHeading, rotatedX, rotatedY, targetPos.position.x, targetPos.position.y));

                double speculatedX = targetPos.position.x - rotatedX;
                double speculatedY = targetPos.position.y - rotatedY;
                Log.i("move to target", String.format("When image is captured, the robot should be at: %4.2f, %4.2f, %4.2f",
                        speculatedX, speculatedY, speculatedHeading));

                Pose2d poseAtCapture = poseAtNanoTime(detection.frameAcquisitionNanoTime);
                Log.i("move to target", String.format("According to history, robot was at: %4.2f, %4.2f, %4.2f",
                        poseAtCapture.position.x, poseAtCapture.position.y, poseAtCapture.heading.toDouble()));

                double currentXEst = current.position.x - poseAtCapture.position.x + speculatedX;
                double currentYEst = current.position.y - poseAtCapture.position.y + speculatedY;
                double currentHeadingEst = utilities.normalizeRadian(current.heading.toDouble() - poseAtCapture.heading.toDouble() + speculatedHeading);
                Log.i("move to target", String.format("Update robot position: %4.2f, %4.2f, %4.2f to %4.2f, %4.2f, %4.2f",
                        current.position.x, current.position.y, current.heading.toDouble(), currentXEst, currentYEst, currentHeadingEst));

                return new Pose2d(new Vector2d(currentXEst, currentYEst), Rotation2d.fromDouble(currentHeadingEst));
            }

            return null;
        }
    }

    public void enableTargetCorrection(Vector2d nearTarget, double nearTargetRange, Pose2d TargetPos) {
        nearTargetCorrection = new NearTargetAdjustment(nearTarget, nearTargetRange, TargetPos);
    }

    public void disableTargetCorrection() {
        nearTargetCorrection = null;
    }

    private final class TrajectoryAccuracy {
        private double timeout;
        private Pose2d error;
        private boolean compareAgainstTarget = false;

        TrajectoryAccuracy(double timeout, Pose2d error, boolean compareAgainstTarget) {
            this.timeout = timeout;
            this.error = error;
            this.compareAgainstTarget = compareAgainstTarget;
        }

        public boolean reachedTarget(Pose2d pos, double finalX, double finalY, double finalHeading) {

            double angleDiff = Utilities.sharedUtility.normalizeRadian(pose.heading.toDouble() - Math.toRadians(finalHeading));
            if (Math.abs(pose.position.x - finalX) < trajectoryAccuracy.error.position.x &&
                    Math.abs(pose.position.y - finalY) < trajectoryAccuracy.error.position.y &&
                    Math.abs(angleDiff) < trajectoryAccuracy.error.heading.toDouble()) {
                Log.v("move to target", String.format("Reached target: at %4.2f, %4.2f, %4.2f, expected: %4.2f, %4.2f, %4.2f",
                        pos.position.x, pos.position.y, pos.heading.toDouble(), finalX, finalY, finalHeading));
                return true;
            }
            return false;
        }

    }
    public void enableTrajectoryAccuracy(double timeout, Pose2d error, boolean compareAgainstTarget) {
        trajectoryAccuracy = new TrajectoryAccuracy(timeout, error, compareAgainstTarget);
    }

    public void disableTrajectoryAccuracy() {
        trajectoryAccuracy = null;
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            Pose2d adjustedPose = null;
            if (nearTargetCorrection != null) {
                adjustedPose = nearTargetCorrection.adjustedPosUsingTarget(pose);
            }
            if (adjustedPose != null) {
                pose = adjustedPose;
            }

            boolean reachedTarget = false;
            double durationExtension = 0.0;
            if (trajectoryAccuracy != null) {

                durationExtension = trajectoryAccuracy.timeout;

                Pose2d poseToCompare = pose;
                if (trajectoryAccuracy.compareAgainstTarget) {
                    poseToCompare = adjustedPose;
                }
                if (poseToCompare != null) {
                    Pose2dDual<Time> finalPos = timeTrajectory.get(timeTrajectory.duration);
                    reachedTarget = trajectoryAccuracy.reachedTarget(poseToCompare,
                            finalPos.position.x.value(), finalPos.position.y.value(), finalPos.heading.value().toDouble());
                    if (!reachedTarget) {
                        durationExtension = trajectoryAccuracy.timeout;
                    }
                }
            }

            if (reachedTarget || t >= timeTrajectory.duration + durationExtension) {
                Log.i("move to target", String.format("Trajectory following will stop now. t: %4.2f, trajectory duration: %4.2f, extension: %4.2f",
                        t, timeTrajectory.duration, durationExtension));
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return false;
            }

            if (t > timeTrajectory.duration) {
                t = timeTrajectory.duration;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            setAdjustedMotorPower(leftFront,  leftFrontPower,  Robot.leftFrontRatio,  Robot.driveMotorPowerMin);
            setAdjustedMotorPower(leftBack,   leftBackPower,   Robot.leftBackRatio,   Robot.driveMotorPowerMin);
            setAdjustedMotorPower(rightBack,  rightBackPower,  Robot.rightBackRatio,  Robot.driveMotorPowerMin);
            setAdjustedMotorPower(rightFront, rightFrontPower, Robot.rightFrontRatio, Robot.driveMotorPowerMin);

            p.put("x_vel", command.linearVel.x.value());
            p.put("y_vel", command.linearVel.y.value());
            p.put("head_vel", command.angVel.value());
            Log.v("drive", String.format("x_vel: %4.2f, y_vel: %4.2f, head_vel: %4.2f", command.linearVel.x.value(), command.linearVel.y.value(), command.angVel.value()));

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            setAdjustedMotorPower(leftFront,  feedforward.compute(wheelVels.leftFront)  / voltage, Robot.leftFrontRatio,  Robot.driveMotorPowerMin);
            setAdjustedMotorPower(leftBack,   feedforward.compute(wheelVels.leftBack)   / voltage, Robot.leftBackRatio,   Robot.driveMotorPowerMin);
            setAdjustedMotorPower(rightBack,  feedforward.compute(wheelVels.rightBack)  / voltage, Robot.rightBackRatio,  Robot.driveMotorPowerMin);
            setAdjustedMotorPower(rightFront, feedforward.compute(wheelVels.rightFront) / voltage, Robot.rightFrontRatio, Robot.driveMotorPowerMin);

            p.put("x_vel", command.linearVel.x.value());
            p.put("y_vel", command.linearVel.y.value());
            p.put("head_vel", command.angVel.value());
            Log.v("drive", String.format("x_vel: %4.2f, y_vel: %4.2f, head_vel: %4.2f", command.linearVel.x.value(), command.linearVel.y.value(), command.angVel.value()));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(new Pose2dWithTime(pose, System.nanoTime()));
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private Pose2d poseAtNanoTime(long nanotime) {
        Pose2dWithTime closestPose = null;
        for (Pose2dWithTime t : poseHistory) {
            if (closestPose == null || Math.abs(t.nanoTime - nanotime) < Math.abs(closestPose.nanoTime - nanotime)) {
                closestPose = t;
            }
        }
        return closestPose.pose;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2dWithTime t : poseHistory) {
            xPoints[i] = t.pose.position.x;
            yPoints[i] = t.pose.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}
*/