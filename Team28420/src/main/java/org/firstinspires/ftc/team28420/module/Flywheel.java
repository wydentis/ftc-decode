package org.firstinspires.ftc.team28420.module;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// TODO: add Niobium's Regression calc
public class Flywheel {
    public static class FlywheelConfig {
        /*** PIDF CONSTANTS ***/
        public static double kF = 30;
        public static double kI = 0;
        public static double kP = 37;
        public static double kD = 2;
        public static double MAX_VEL = 1000;
    }

    /*** HARDWARE ***/
    private final DcMotorEx left, right;

    public Flywheel(HardwareMap hMap) {
        this.left = hMap.get(DcMotorEx.class, "shLeft");
        this.right = hMap.get(DcMotorEx.class, "shRight");
    }

    public void setup() {
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets coefficient to the maximum speed of shooter wheels
     *
     * @param k
     */
    public void setVelocityCoefficient(double k) {
        double desired = FlywheelConfig.MAX_VEL * k;

        left.setVelocity(desired);
        right.setVelocity(desired);
    }

    /**
     * Converts ticks per second to revolutions per minute
     * Useful for debugging
     *
     * @param tps
     * @return rpm
     */
    private double toRPM(double tps) {
        return tps * 60.0 / 28.0;
    }

    private void setMotorsMode(DcMotor.RunMode mode) {
        left.setMode(mode);
        right.setMode(mode);
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("flywheel left velocity", left.getVelocity());
        telemetry.addData("flywheel left RPM", toRPM(left.getVelocity()));

        telemetry.addData("flywheel right velocity", right.getVelocity());
        telemetry.addData("flywheel right RPM", toRPM(right.getVelocity()));
    }
}
