package org.firstinspires.ftc.team28420.module;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Revolver {
    public enum RevolverState {IDLE, REVOLVER_TURNING}

    /*** CONFIG CONSTANTS ***/
    public static double SORT_MOTOR_TICKS_PER_TURN = 1075.0;
    public static double MAX_VEL = 2000; // Ticks per second
    public static double MAX_ACCEL = 1500; // Ticks per second squared

    /*** PIDF CONSTANTS ***/
    public static double kP = 0.005;
    public static double kV = 1.0 / MAX_VEL; // Feedforward Velocity
    public static double kA = 0.0001;        // Feedforward Acceleration


    /*** MOTION PROFILING ***/
    private double profileStartTime = 0;
    private double startPosition = 0;
    private double globalTarget = 0;

    /*** HARDWARE ***/
    private final DcMotorEx revolver;

    /*** TIMERS ***/
    private final ElapsedTime profileTimer = new ElapsedTime();
    private RevolverState state = RevolverState.IDLE;

    public Revolver(HardwareMap hMap) {
        this.revolver = hMap.get(DcMotorEx.class, "sort");
    }


    /**
     * Responsible for resetting all the internal variables into their initial state.
     */
    public void setup() {
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        state = RevolverState.IDLE;
        profileStartTime = 0;
        startPosition = 0;
        globalTarget = 0;
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *  Gives proportional power for current profile of revolver rotation.
     *  It uses motion profiling with 3 steps: acceleration, cruising, deceleration.
     * @return power needed on current step of rotation
     */
    private double calculateProfilePower() {
        double timeElapsed = profileTimer.seconds() - profileStartTime;
        double distance = globalTarget - startPosition;
        if (Math.abs(distance) < 2) return 0;

        double accelTime = MAX_VEL / MAX_ACCEL;
        double accelDist = 0.5 * MAX_ACCEL * Math.pow(accelTime, 2);

        double targetVel = 0;
        double targetAccel = 0;
        double profilePos = startPosition;

        if (Math.abs(distance) > 2 * accelDist) {
            double cruiseDist = Math.abs(distance) - (2 * accelDist);
            double cruiseTime = cruiseDist / MAX_VEL;

            if (timeElapsed < accelTime) {
                targetVel = MAX_ACCEL * timeElapsed;
                targetAccel = MAX_ACCEL;
                profilePos = startPosition + Math.signum(distance) * (0.5 * MAX_ACCEL * Math.pow(timeElapsed, 2));
            } else if (timeElapsed < accelTime + cruiseTime) {
                targetVel = MAX_VEL;
                profilePos = startPosition + Math.signum(distance) * (accelDist + MAX_VEL * (timeElapsed - accelTime));
            } else if (timeElapsed < 2 * accelTime + cruiseTime) {
                double decelTime = timeElapsed - accelTime - cruiseTime;
                targetVel = MAX_VEL - (MAX_ACCEL * decelTime);
                targetAccel = -MAX_ACCEL;
                profilePos = startPosition + Math.signum(distance) * (accelDist + cruiseDist + (MAX_VEL * decelTime) - (0.5 * MAX_ACCEL * Math.pow(decelTime, 2)));
            } else {
                profilePos = globalTarget;
            }
        } else {
            profilePos = globalTarget;
        }

        double currentPos = revolver.getCurrentPosition();
        return (kV * targetVel) + (kA * targetAccel) + (kP * (profilePos - currentPos));
    }

    public boolean isBusy() {
        return state == RevolverState.REVOLVER_TURNING;
    }

    /**
     * @return Current actual angle
      */
    public double currentAngle() {
        return revolver.getCurrentPosition() / SORT_MOTOR_TICKS_PER_TURN * 360.0;
    }

    /**
     * Rotates revolver by given degrees
     * @param deg - degrees to turn
     */
    public void rotateRevolver(double deg) {
        double ticksToMove = deg * SORT_MOTOR_TICKS_PER_TURN / 360.0;
        rotateRevolverTicks(ticksToMove);
    }

    private void rotateRevolverTicks(double ticks) {
        startPosition = revolver.getCurrentPosition();
        globalTarget = startPosition + ticks;

        profileStartTime = profileTimer.seconds();
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        state = RevolverState.REVOLVER_TURNING;
    }

    /**
     * Updating state behaviour of revolver
     */
    public void update() {
        switch (state) {
            case REVOLVER_TURNING:
                double power = calculateProfilePower();
                revolver.setPower(power);

                double accelTime = MAX_VEL / MAX_ACCEL;
                double cruiseTime = (Math.abs(globalTarget - startPosition) - (MAX_VEL / MAX_ACCEL * MAX_VEL)) / MAX_VEL;
                if (profileTimer.seconds() - profileStartTime > (2 * accelTime + Math.max(0, cruiseTime)) + 0.1) {
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
    }


}
