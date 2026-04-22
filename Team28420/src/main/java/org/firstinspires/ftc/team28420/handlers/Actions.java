package org.firstinspires.ftc.team28420.handlers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team28420.module.CachedIMU;
import org.firstinspires.ftc.team28420.module.Dribbler;
import org.firstinspires.ftc.team28420.module.Flywheel;
import org.firstinspires.ftc.team28420.module.Movement;
import org.firstinspires.ftc.team28420.module.Odometry;
import org.firstinspires.ftc.team28420.module.RobotController;
import org.firstinspires.ftc.team28420.types.AprilTag;
import org.firstinspires.ftc.team28420.types.MovementParams;
import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.Position;

public class Actions {

    private final RobotController robotController;
    private final Movement movement;
    private final Odometry odometry;
    private final CachedIMU imu;
    private final CameraHandler cameraHandler;
    private final IntakeHandler intakeHandler;
    private final Flywheel flywheel;
    private final Dribbler dribbler;
    private final AprilTag targetTag;

    public Actions(HardwareMap hMap, AprilTag targetTag) {
        this.robotController = new RobotController(hMap);
        this.movement = new Movement(hMap);
        this.odometry = new Odometry(hMap);
        this.imu = new CachedIMU(hMap);
        this.cameraHandler = new CameraHandler(hMap);
        this.intakeHandler = new IntakeHandler(hMap);
        this.flywheel = new Flywheel(hMap);
        this.dribbler = new Dribbler(hMap);
        this.targetTag = targetTag;
    }

    public static float getCubic(float axis) {
        return (float) Math.pow(axis, 3);
    }

    public static float withDeathzone(float axis, float threshold) {
        return Math.abs(axis) < threshold ? 0 : axis;
    }

    public static Telemetry getFTCDashboardTelemetry(Telemetry baseTelemetry) {
        return new MultipleTelemetry(baseTelemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void setup() {
        robotController.setManualCachingMode();
        movement.setup();
        odometry.setup();
        imu.setup();
        cameraHandler.setup();
        intakeHandler.setup();
        flywheel.setup();
    }

    public void update() {
        robotController.clearCache();
        odometry.update();
        imu.update();
        cameraHandler.update();
        intakeHandler.update();
        flywheel.setVelocityCoefficient(1.5);

        setMotif();
    }

    public void shoot() {
        intakeHandler.shoot();
    }

    public void shootWithCheck() {
        if (!odometry.getRobotPosition().isValidForShoot()) return;
        shoot();
    }

    public void moveByParamsWithOffset(MovementParams params, double angleOffset) {
        params.getMoveVector().rotate(angleOffset);
        moveWithFixByParams(params);
    }

    public void moveToAprilTag() {
        PolarVector vector = getBestPosition().getVectorToNearestValidShootPoint();
        moveWithFixByParams(new MovementParams(vector, getAngleForAprilTag()));
    }

    public void brake() {
        movement.brake();
    }

    public void log(Telemetry telemetry) {
        movement.log(telemetry);
        odometry.log(telemetry);
        imu.log(telemetry);
        cameraHandler.log(telemetry);
        intakeHandler.log(telemetry);
        flywheel.log(telemetry);
        dribbler.log(telemetry);
    }

    private void setMotif() {
        if (intakeHandler.getTargetMotif() == null) {
            intakeHandler.setTargetMotif(cameraHandler.getMotif());
        }
    }

    public void setDribblerState(boolean dribble, boolean invert) {
        if (dribble && invert) {
            dribbler.setDribblerState(Dribbler.DribblerState.DROP);
        } else if (dribble) {
            dribbler.setDribblerState(Dribbler.DribblerState.INTAKE);
        } else {
            dribbler.setDribblerState(Dribbler.DribblerState.IDLE);
        }
    }

    private void moveWithFixByParams(MovementParams params) {
        params.getMoveVector().rotate(-odometry.getRobotHeading());
        movement.moveByParams(params);
    }

    private Position getBestPosition() {
        Position cameraPosition = cameraHandler.getRobotPosition(targetTag);
        if (cameraPosition != null) {
            return cameraPosition;
        }
        return odometry.getRobotPosition();
    }

    private double getAngleForAprilTag() {
        Double cameraAngle = cameraHandler.getAprilTagBearing(targetTag);
        if (cameraAngle != null) {
            return cameraAngle;
        }
        return odometry.getRobotPosition().getAngleTo(targetTag.getPosition());
    }
}
