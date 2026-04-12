package org.firstinspires.ftc.team28420.module;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.team28420.config.BallDetectionConf;
import org.firstinspires.ftc.team28420.config.CameraConf;
import org.firstinspires.ftc.team28420.config.CameraServoConf;
import org.firstinspires.ftc.team28420.config.GyroConf;
import org.firstinspires.ftc.team28420.config.ShooterConf;
import org.firstinspires.ftc.team28420.config.WheelBaseConf;
import org.firstinspires.ftc.team28420.module.shooter.Shooter;
import org.firstinspires.ftc.team28420.processors.BallDetection;
import org.firstinspires.ftc.team28420.types.AprilTag;
import org.firstinspires.ftc.team28420.types.MovementParams;
import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.Position;
import org.firstinspires.ftc.team28420.types.ShooterCalculationMode;
import org.firstinspires.ftc.team28420.types.WheelsRatio;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Point;

import java.util.List;

public class Actions {

    private final Movement mv;
    private final IMU imu;
    private final Camera cam;

    private final Servo cameraServo;
    private final Shooter shooter;
    private final Parking parking;
    private final BallDetection ballDetection;
    private final Telemetry telemetry;
    double rpm;

    private YawPitchRollAngles lastAngles = new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0);
    private List<LynxModule> allHubs;
    private double lastValidDistance = 1.5;
    public Actions(HardwareMap hMap, boolean isAuto, Telemetry telemetry) throws InterruptedException {
        this.mv = new Movement(hMap);
        this.imu = hMap.get(IMU.class, GyroConf.IMU);
        if(isAuto)
            this.ballDetection = new BallDetection();
        else
            this.ballDetection = null;
        this.cam = new Camera(hMap, ballDetection);
        this.shooter = new Shooter(hMap, telemetry);
        this.cameraServo = hMap.get(Servo.class, "cameraServo");
        this.parking = new Parking(hMap);
        allHubs = hMap.getAll(LynxModule.class);
        this.telemetry = telemetry;
    }

    public void init() {
        mv.setup();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(GyroConf.logoFacingDirection, GyroConf.usbFacingDirection)));
        shooter.setup();
        parking.setup();
        camIdle();
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void camPeek() {
        cameraServo.setPosition(CameraServoConf.PEEK_POS);
    }
    public void camIdle() {
        cameraServo.setPosition(CameraServoConf.IDLE_POS);
    }

    public void brake() {
        mv.brake();
    }

    public void updateShooter() {
        shooter.update();
    }

    public void toggleShooterManualControl(boolean active) {
        shooter.toggleManualControl(active);
    }

    public void setDribblerVelocityCoefficient(float k) {
        shooter.setDribblerVelocityCoefficient(k);
    }

    public void setHelperWheelCoefficient(float k) {
        shooter.setHelperWheelCoefficient(k);
    }

    //todo: remove setVelocityCoefficient if not used
    /**
     * Prepares the shooter by calculating and setting the required velocity.
     *
     * @param distance current distance to the target
     */
    public void prepareForShoot(double distance) {
        if (distance > 0) {
            lastValidDistance = distance;
        }

        double rpm = calculateTargetRpm(lastValidDistance);
        shooter.setVelocity(rpm);
    }

    /**
     * Dispatches the calculation based on the selected mode.
     *
     * @param distance distance to the target
     * @return calculated target RPM
     */
    private double calculateTargetRpm(double distance) {
        if (ShooterConf.SHOOTER_CALCULATION_MODE == ShooterCalculationMode.TABLE) {
            return shooter.calculateRpmByTable(distance);
        }
        return shooter.calculateRpmByRegression(distance);
    }

    public void afterStart() {
        shooter.afterStart();
    }

    public boolean shoot() {
        return shooter.shoot();
    }

    public void revolverRotate(double deg) {
        shooter.rotateRevolver(deg);
        shooter.toggleManualControl(true);
    }

    public void resetRevolverTicks() {
        shooter.resetRevolverTicks();
    }

    public void alignRevolverToTarget() {
        shooter.alignRevolverToTarget();
    }

    public void setDefaultAutoMotif(String motif) {
        for (char color : motif.toCharArray()) {
            shooter.appendBallToMotif(color);
        }
    }

    public boolean isShootable() {
        return shooter.isShootable();
    }

    public YawPitchRollAngles getRobotAngles() {
        return lastAngles;
    }

    public void updateLastAngles() {
        lastAngles = imu.getRobotYawPitchRollAngles();
    }

    public void move(WheelsRatio<Double> ratio) {
        mv.setMotorsVelocityRatios(ratio, WheelBaseConf.MAX_VELOCITY);
    }

    public WheelsRatio<Double> getRatios(MovementParams params) {
        return Movement.vectorToRatios(params);
    }

    public WheelsRatio<Double> getRatios(double x, double y, double rx) {
        return Movement.vectorToRatios(new MovementParams(new PolarVector(new Position(x, y)), rx));
    }

    public void updateApriltags() {
        cam.updateApriltags();
    }

    public WheelsRatio<Double> getRatiosForApriltag(AprilTag tag, double offsetX, double offsetY) {
        AprilTagDetection detection = cam.getAprilTagDetection(tag);
        MovementParams params = cam.getMovementParamsToPoint(detection, offsetX, offsetY);
        return Movement.vectorToRatios(params);
    }

    public WheelsRatio<Double> getRatiosLookApriltag(AprilTag tag, double offsetX, double offsetY) {
        AprilTagDetection detection = cam.getAprilTagDetection(tag);
        MovementParams params = cam.getMovementParamsToOffset(detection, offsetX, offsetY);
        return Movement.vectorToRatios(params);
    }

    public float getDistanceToAprilTag(AprilTag aprilTag) {
        AprilTagDetection detection = cam.getAprilTagDetection(aprilTag);
        if (detection != null && detection.ftcPose != null) {
            return (float) detection.ftcPose.y;
        }

        return -1.0f;
    }

    public void park() {
        parking.park();
    }

    public void setMotif() {
        AprilTagDetection detection = cam.getAprilTagDetection(AprilTag.GREEN);
        if (detection != null) {
            ShooterConf.TARGET_MOTIF = AprilTag.getMotif(detection.id);
        }
    }

    public double getCubic(double axis) {
        return Math.pow(axis, 3);
    }

    public double withDeathzone(double axis, double threshold) {
        return Math.abs(axis) < threshold ? 0 : axis;
    }

    public void log() {
        cam.log(telemetry);
        shooter.log(telemetry);
        telemetry.addData("yaw", getRobotAngles().getYaw(AngleUnit.RADIANS));
        if(ballDetection != null)
            ballDetection.updateTelemetry(telemetry);
    }

    public Point getDetectedBallPosition() {
        return ballDetection.getBallPosition();
    }

    public void setShooterPids() {
        shooter.setPids();
    }

    public void aimAndDriveToBall() {
        Point ballPos = getDetectedBallPosition();

        if (ballPos != null) {
            double errorX = ballPos.x - CameraConf.WIDTH/2.0;
            double rx = errorX * BallDetectionConf.kP;
            move(getRatios(0, 0.067, rx));
        } else {
            move(getRatios(0, 0.08, 0));
        }
    }

    public double getForceToGyro(double angle) {
        telemetry.addData("rotate force", (getRobotAngles().getYaw(AngleUnit.RADIANS) - angle) / Math.PI);
        return (getRobotAngles().getYaw(AngleUnit.RADIANS) - angle) / Math.PI;
    }

    public void setScanAllowed(boolean b) {
        shooter.setScanAllowed(b);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void clearCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
