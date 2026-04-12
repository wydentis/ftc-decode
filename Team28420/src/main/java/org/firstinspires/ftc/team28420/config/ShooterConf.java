package org.firstinspires.ftc.team28420.config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.team28420.module.shooter.InterpolationTable;
import org.firstinspires.ftc.team28420.types.ShooterCalculationMode;
import org.opencv.core.Scalar;

@Config
public class ShooterConf {
    public static boolean IS_AUTO = false;
    public static double REVOLVER_VELOCITY = 1331;
    public static double SORT_P = 6.2;
    public static Scalar cslowGreen = new Scalar(70, 0.4, 0.0019);
    public static Scalar cshighGreen = new Scalar(180, 1, 0.01);
    public static Scalar cslowPurple = new Scalar(240, 0.3, 0.006);
    public static Scalar cshighPurple = new Scalar(330, 1, 0.04);
    public static double SCANNED_BALL_MS = 70;
    public static String TARGET_MOTIF = null;
    public static double BALL_DETECTION_THRESHOLD = 4;
    public static double SHOOTER_F = 30;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_P = 37;
    public static double SHOOTER_D = 2;
    public static int VELOCITY = 522;
    public static int DRIBBLER_VELOCITY = 2067;
    public static double SORT_MOTOR_TICKS_PER_TURN = 1067.0; // TODO: calibrate
    public static double STALL_THRESHOLD_TPS = 80.0;
    public static double STALL_TIMEOUT_SEC = 1;
    public static int BUSY_TOLERANCE_TICKS = 25;

    //todo: calculate coefficients
    public static final double SHOOTER_A = 150.0;
    public static final double SHOOTER_B = 10.0;
    public static final double SHOOTER_C = 600.0;
    public static final double MIN_SPEED = 0.0;
    public static final double MAX_SPEED = 4000.0;

    //todo: calculate distances and velocities
    public static final InterpolationTable SHOOTER_TABLE = new InterpolationTable()
            .addPoint(1.0, 1500.0)
            .addPoint(1.5, 1800.0)
            .addPoint(2.0, 2200.0)
            .addPoint(2.5, 2600.0)
            .addPoint(3.0, 2900.0);
    public static final ShooterCalculationMode SHOOTER_CALCULATION_MODE = ShooterCalculationMode.REGRESSION;
}
