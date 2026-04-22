package org.firstinspires.ftc.team28420.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Revolver {
    public static double Kp = 0.0052;
    public static double Kd = 0.0001;

    public enum RevolverState {IDLE, REVOLVER_TURNING}

    /*** CONFIG ***/
    public static double SORT_MOTOR_TICKS_PER_TURN = 1074.0;
    public static double MAX_POWER = 0.8;

    /*** HARDWARE ***/
    private final DcMotorEx revolver;

    /*** STATE ***/
    private RevolverState state = RevolverState.IDLE;

    private double targetTicks = 0.0;

    public Revolver(HardwareMap hMap) {
        this.revolver = hMap.get(DcMotorEx.class, "sort");
    }

    public void setup() {
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetTicks = revolver.getCurrentPosition();
        revolver.setTargetPosition((int)targetTicks);
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        revolver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        revolver.setPower(0.0);

        state = RevolverState.IDLE;
    }

    public boolean isBusy() {
        return state == RevolverState.REVOLVER_TURNING;
    }

    public double currentAngle() {
        return revolver.getCurrentPosition() / SORT_MOTOR_TICKS_PER_TURN * 360.0;
    }

    /**
     * Повернуть на относительный угол в градусах.
     */
    public void rotateRevolver(double deg) {
        targetTicks += deg * SORT_MOTOR_TICKS_PER_TURN / 360.0;
        revolver.setTargetPosition((int)Math.round(targetTicks));
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        state = RevolverState.REVOLVER_TURNING;
    }

    public void update() {
        switch (state) {
            case REVOLVER_TURNING:
                double currentPos = revolver.getCurrentPosition();
                double error = targetTicks - currentPos;
                double velocity = revolver.getVelocity();

                double power = (error * Kp) - (velocity * Kd);

                if (Math.abs(power) > 0.6 && Math.abs(velocity) < 10) {
                    power = 0;
                    state = RevolverState.IDLE;
                }
                revolver.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));

                if (Math.abs(error) < 10 && Math.abs(velocity) < 5) {
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
        telemetry.addData("REVOLVER STATE", state);
    }
}