package org.firstinspires.ftc.team28420.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Revolver {
    public enum RevolverState {IDLE, REVOLVER_TURNING}

    /*** CONFIG CONSTANTS ***/
    public static double SORT_MOTOR_TICKS_PER_TURN = 1075.0;
    public static double MAX_VEL = 2100; // ticks/sec
    public static double MAX_ACCEL = 4400; // ticks/sec^2

    /*** PIDF CONSTANTS ***/
    public static double kP = 0.0052;
    public static double kV = 1.0 / MAX_VEL;
    public static double kA = 0.00004;

    /*** FINISH THRESHOLDS ***/
    public static double POSITION_TOL_TICKS = 6.0;
    public static double VELOCITY_TOL_TPS = 22.0;
    public static double MIN_PROFILE_TIME = 0.05; // sec

    /*** MOTION PROFILING ***/
    private double profileStartTime = 0;
    private double startPosition = 0;
    private double targetTicks = 0;
    private double plannedTotalTime = 0;

    /*** HARDWARE ***/
    private final DcMotorEx revolver;

    /*** TIMERS ***/
    private final ElapsedTime profileTimer = new ElapsedTime();
    private RevolverState state = RevolverState.IDLE;

    public Revolver(HardwareMap hMap) {
        this.revolver = hMap.get(DcMotorEx.class, "sort");
    }

    public void setup() {
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        state = RevolverState.IDLE;
        profileStartTime = 0;
        startPosition = 0;
        targetTicks = 0;
        plannedTotalTime = 0;
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revolver.setPower(0);
        revolver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double calculateProfilePower() {
        double t = profileTimer.seconds() - profileStartTime;
        double distance = targetTicks - startPosition;
        double sgn = Math.signum(distance);
        double distAbs = Math.abs(distance);

        if (distAbs < 1e-6) return 0.0;

        double accelTimeMax = MAX_VEL / MAX_ACCEL;
        double accelDistMax = 0.5 * MAX_ACCEL * accelTimeMax * accelTimeMax;

        double targetVel;
        double targetAccel;
        double profilePos;

        // Check if the profile has completely finished
        boolean isFinished = false;

        if (distAbs >= 2.0 * accelDistMax) {
            // Trapezoid
            double cruiseDist = distAbs - 2.0 * accelDistMax;
            double cruiseTime = cruiseDist / MAX_VEL;
            double totalTime = 2.0 * accelTimeMax + cruiseTime;

            if (t < 0) t = 0;
            if (t >= totalTime) {
                t = totalTime;
                isFinished = true;
            }

            if (t < accelTimeMax) {
                targetVel = MAX_ACCEL * t;
                targetAccel = MAX_ACCEL;
                profilePos = startPosition + sgn * (0.5 * MAX_ACCEL * t * t);
            } else if (t < accelTimeMax + cruiseTime) {
                double tc = t - accelTimeMax;
                targetVel = MAX_VEL;
                targetAccel = 0;
                profilePos = startPosition + sgn * (accelDistMax + MAX_VEL * tc);
            } else {
                double td = t - accelTimeMax - cruiseTime;
                targetVel = MAX_VEL - MAX_ACCEL * td;
                targetAccel = -MAX_ACCEL;
                profilePos = startPosition + sgn *
                        (accelDistMax + cruiseDist + MAX_VEL * td - 0.5 * MAX_ACCEL * td * td);
            }
        } else {
            // Triangular
            double accelTime = Math.sqrt(distAbs / MAX_ACCEL);
            double peakVel = MAX_ACCEL * accelTime;
            double totalTime = 2.0 * accelTime;

            if (t < 0) t = 0;
            if (t >= totalTime) {
                t = totalTime;
                isFinished = true;
            }

            if (t < accelTime) {
                targetVel = MAX_ACCEL * t;
                targetAccel = MAX_ACCEL;
                profilePos = startPosition + sgn * (0.5 * MAX_ACCEL * t * t);
            } else {
                double td = t - accelTime;
                targetVel = peakVel - MAX_ACCEL * td;
                targetAccel = -MAX_ACCEL;
                double accelDist = 0.5 * MAX_ACCEL * accelTime * accelTime;
                profilePos = startPosition + sgn *
                        (accelDist + peakVel * td - 0.5 * MAX_ACCEL * td * td);
            }
        }

        // Fix 2: If the profile is done, we shouldn't have any residual acceleration forces
        if (isFinished) {
            targetVel = 0;
            targetAccel = 0;
            profilePos = targetTicks; // lock exactly to target
        }

        // Fix 1: Apply the sign to the feedforward velocities and accelerations
        double currentPos = revolver.getCurrentPosition();
        return (kV * targetVel * sgn) + (kA * targetAccel * sgn) + (kP * (profilePos - currentPos));
    }

    public boolean isBusy() {
        return state == RevolverState.REVOLVER_TURNING;
    }

    public double currentAngle() {
        return revolver.getCurrentPosition() / SORT_MOTOR_TICKS_PER_TURN * 360.0;
    }

    public void rotateRevolver(double deg) {
        if (Math.abs(deg) < 0.1) {
            return;
        }
        double deltaTicks = deg * SORT_MOTOR_TICKS_PER_TURN / 360.0;

        double newTarget = targetTicks + deltaTicks;

        startMotionProfile(newTarget);
    }
    private void startMotionProfile(double target) {
        startPosition = revolver.getCurrentPosition();
        targetTicks = target;
        profileStartTime = profileTimer.seconds();

        double distAbs = Math.abs(targetTicks - startPosition);
        if (distAbs < 5.0) {
            state = RevolverState.IDLE;
            return;
        }

        double accelTimeMax = MAX_VEL / MAX_ACCEL;
        double accelDistMax = 0.5 * MAX_ACCEL * accelTimeMax * accelTimeMax;

        if (distAbs >= 2.0 * accelDistMax) {
            plannedTotalTime = 2.0 * accelTimeMax + (distAbs - 2.0 * accelDistMax) / MAX_VEL;
        } else {
            plannedTotalTime = 2.0 * Math.sqrt(distAbs / MAX_ACCEL);
        }

        state = RevolverState.REVOLVER_TURNING;
    }

    public void update() {
        switch (state) {
            case REVOLVER_TURNING:
                revolver.setPower(calculateProfilePower());

                double err = Math.abs(targetTicks - revolver.getCurrentPosition());
                double vel = Math.abs(revolver.getVelocity());
                double elapsed = profileTimer.seconds() - profileStartTime;

                if ((err <= POSITION_TOL_TICKS && vel <= VELOCITY_TOL_TPS && elapsed > MIN_PROFILE_TIME)
                        || elapsed > plannedTotalTime + 0.35) {
                    revolver.setPower(0);
                    state = RevolverState.IDLE;
                }
                break;

            case IDLE:
                revolver.setPower(0);
                break;
        }
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("CURRENT REVOLVER TICKS", revolver.getCurrentPosition());
        telemetry.addData("REVOLVER SPEED TPS", revolver.getVelocity());
        telemetry.addData("ANGLE", currentAngle());
        telemetry.addData("REVOLVER TARGET", targetTicks);
        telemetry.addData("REVOLVER BUSY", isBusy());
    }
}
