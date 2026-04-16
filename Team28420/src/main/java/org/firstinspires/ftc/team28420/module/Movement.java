package org.firstinspires.ftc.team28420.module;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.team28420.config.WheelBaseConf;
import org.firstinspires.ftc.team28420.types.MovementParams;
import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.WheelsRatio;

public class Movement {
    private final DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private double currentLF = 0.0;
    private double currentRF = 0.0;
    private double currentLB = 0.0;
    private double currentRB = 0.0;
    public static double p = 0.095;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 11.7;

    public static WheelsRatio<Double> vectorToRatios(MovementParams params) {
        double theta = params.getMoveVector().getTheta();
        double power = params.getMoveVector().getAbs();
        double turn = params.getTurnAbs();

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double lf = power * cos / max + turn;
        double rf = power * sin / max - turn;
        double lb = power * sin / max + turn;
        double rb = power * cos / max - turn;

        double scale = Math.max(1.0, Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb))
        ));

        return new WheelsRatio<>(lf / scale, rf / scale, lb / scale, rb / scale);
    }

    public Movement(HardwareMap hMap) {
        this.leftFront = hMap.get(DcMotorEx.class, WheelBaseConf.LEFT_TOP_MOTOR);
        this.rightFront = hMap.get(DcMotorEx.class, WheelBaseConf.RIGHT_TOP_MOTOR);
        this.leftBack = hMap.get(DcMotorEx.class, WheelBaseConf.LEFT_BOTTOM_MOTOR);
        this.rightBack = hMap.get(DcMotorEx.class, WheelBaseConf.RIGHT_BOTTOM_MOTOR);
    }

    public void setup() {
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfNew = new PIDFCoefficients(p, i, d, f);

        leftFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        rightFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        leftBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        rightBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
    }

    public void setMotorsMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    public void setMotorsTargetPosition(WheelsRatio<Double> wheelsRatio) {
        WheelsRatio<Integer> wheelsRatioInteger = wheelsRatio.toInt(1);
        leftFront.setTargetPosition(wheelsRatioInteger.getLeftTop());
        rightFront.setTargetPosition(wheelsRatioInteger.getRightTop());
        leftBack.setTargetPosition(wheelsRatioInteger.getLeftBottom());
        rightBack.setTargetPosition(wheelsRatioInteger.getRightBottom());
    }

    public void setMotorsPowerRatios(WheelsRatio<Double> wheelsRatio) {
        leftFront.setPower(wheelsRatio.getLeftTop());
        rightFront.setPower(wheelsRatio.getRightTop());
        leftBack.setPower(wheelsRatio.getLeftBottom());
        rightBack.setPower(wheelsRatio.getRightBottom());
    }

    public void setMotorsVelocityRatios(WheelsRatio<Double> wheelsRatio, int velocityMult) {
        leftFront.setVelocity(wheelsRatio.getLeftTop() * velocityMult);
        rightFront.setVelocity(wheelsRatio.getRightTop() * velocityMult);
        leftBack.setVelocity(wheelsRatio.getLeftBottom() * velocityMult);
        rightBack.setVelocity(wheelsRatio.getRightBottom() * velocityMult);
    }

    public void brake() {
        setMotorsPowerRatios(WheelsRatio.ZERO);
    }
}
