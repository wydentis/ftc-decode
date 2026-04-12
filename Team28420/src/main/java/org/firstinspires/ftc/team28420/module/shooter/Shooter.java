package org.firstinspires.ftc.team28420.module.shooter;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team28420.config.ShooterConf;
import org.firstinspires.ftc.team28420.module.Dribbler;
import org.firstinspires.ftc.team28420.processors.BallDetection;
import org.opencv.core.Scalar;

public class Shooter {
    private final DcMotorEx left, right, revolver;
    private final ColorSensor cs;
    private final BallDetection.BallColor color = null;
    private final ElapsedTime shooterTime = new ElapsedTime();
    private final ElapsedTime debounceTimer = new ElapsedTime();
    private final Dribbler dribbler;
    private final Pusher pusher;
    private int globalTarget = 0;
    private ShooterState state = ShooterState.IDLE;
    private boolean manualControl = false;

    private boolean scanAllowed = false;
    private boolean ballPresent = false;
    private boolean potentialBallDetected = false;
    private boolean isUnjamming = false;
    private int originalTargetBeforeJam = 0;
    private final ElapsedTime stallTimer = new ElapsedTime();
    private MotifSorter sorter = new MotifSorter();
    private Telemetry telemetry;

    private boolean shot = false;

    public Shooter(HardwareMap hMap, Telemetry telemetry) {
        this.left = hMap.get(DcMotorEx.class, "shLeft");
        this.right = hMap.get(DcMotorEx.class, "shRight");
        this.revolver = hMap.get(DcMotorEx.class, "sort");
        this.cs = hMap.get(ColorSensor.class, "colorSensor");

        pusher = new Pusher(hMap);
        dribbler = new Dribbler(hMap);

        this.telemetry = telemetry;
    }

    public void setScanAllowed(boolean allowed) {
        scanAllowed = allowed;
    }

    public void setup() {
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        revolver.setTargetPosition(0);
        globalTarget = 0;

        pusher.setState(Pusher.PusherState.NEUTRAL);
        setPids();

        //setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void afterStart() {
        syncTicks();
        pusher.setState(Pusher.PusherState.NEUTRAL);
    }

    /**
     * Custom busy check because DcMotor.isBusy() is often unreliable
     * for detecting if a motor has actually finished its path.
     */
    private boolean isMotorBusy(DcMotorEx motor) {
        if (true) return motor.isBusy();
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > ShooterConf.BUSY_TOLERANCE_TICKS;
    }

    private void handleRevolverStall() {
        if (state != ShooterState.REVOLVER_TURNING) {
            stallTimer.reset();
            return;
        }

        boolean motorTryingToMove = isMotorBusy(revolver);

        if (motorTryingToMove && !isUnjamming) {
            // If we are trying to move but velocity is near zero
            if (Math.abs(revolver.getVelocity()) < ShooterConf.STALL_THRESHOLD_TPS) {
                if (stallTimer.seconds() > ShooterConf.STALL_TIMEOUT_SEC) {
                    // JAM DETECTED
                    isUnjamming = true;
                    originalTargetBeforeJam = globalTarget;

                    double dir =Math.signum(revolver.getVelocity());

                    // Back up slightly (e.g., 45 degrees) to clear the jam
                    // We use a relative move from CURRENT position, not global target
                    globalTarget = revolver.getCurrentPosition() - (int)(60.0 * dir * ShooterConf.SORT_MOTOR_TICKS_PER_TURN / 360.0);
                    revolver.setTargetPosition(globalTarget);
                    revolver.setVelocity(ShooterConf.REVOLVER_VELOCITY);
                }
            } else {
                stallTimer.reset();
            }
        } else {
            stallTimer.reset();
        }

        // If we were unjamming and reached the "back-off" position
        if (isUnjamming && !isMotorBusy(revolver)) {
            isUnjamming = false;
            // Resume original target
            globalTarget = originalTargetBeforeJam;
            revolver.setTargetPosition(globalTarget);
            revolver.setVelocity(ShooterConf.REVOLVER_VELOCITY);
        }

        telemetry.addData("Stall Timer", stallTimer.seconds());
        telemetry.addData("Is Unjamming", isUnjamming);
    }

    public void update() {
        //handleRevolverStall();

        switch (state) {
            case SHOOTING:
                if (shooterTime.milliseconds() >= 225) {
                    pushBall(false);
                    state = Shooter.ShooterState.STOP_SHOOTING;
                    shooterTime.reset();
                }
                break;
            case STOP_SHOOTING:
                if (shooterTime.milliseconds() >= 175) {
                    if(manualControl) {
                        rotateRevolver(120);
                    } else {
                        sortedNextBall();
                        state = ShooterState.REVOLVER_TURNING;
                    }
                    shooterTime.reset();
                }
                break;
            case REVOLVER_TURNING:
                if (!isMotorBusy(revolver)) {
                    state = Shooter.ShooterState.IDLE;
                    shooterTime.reset();
                }
                break;

            case IDLE:
                if (shouldScan()) scanBall();
                break;
        }
    }
    public void sortedNextBall() {
        if (!sorter.getCurMotif().isEmpty()) {
            sorter.dropLastBall();

            if (!sorter.getCurMotif().isEmpty()) rotateRevolver(120);
            else {
                rotateRevolver(60);
                sorter.setCorrectMotif(false);
            }
        } else shot = false;
    }
    public void rotateRevolver(double deg) {
        globalTarget += (int) (deg * ShooterConf.SORT_MOTOR_TICKS_PER_TURN / 360.0);
        revolver.setTargetPosition(globalTarget);
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        revolver.setVelocity(ShooterConf.REVOLVER_VELOCITY);
        state = ShooterState.REVOLVER_TURNING;
        shot = false;
        stallTimer.reset(); // Reset timer when starting a new movement
    }

    public void toggleManualControl(boolean active) {
        if (state == ShooterState.IDLE) {
            if (!active && manualControl) {
                snapToNearestSlot();
            }
            if (active != manualControl) {
                manualControl = active;
                sorter.resetMotif();
            }
        }
    }

    public void setDribblerVelocityCoefficient(float k) {
        dribbler.setVelocityCoefficient(k);
    }

    public void setHelperWheelCoefficient(float k) {
        right.setVelocity(ShooterConf.VELOCITY * k);
    }

    public void setMotorsMode(DcMotor.RunMode mode) {
        left.setMode(mode);
        right.setMode(mode);
        revolver.setMode(mode);
    }

    public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        left.setZeroPowerBehavior(behavior);
        right.setZeroPowerBehavior(behavior);
        //revolver.setZeroPowerBehavior(behavior);
    }

    private void syncTicks() {
        globalTarget = 0;
        revolver.setTargetPosition(0);
    }

    private double toRPM(double tps) {
        return tps * 60.0 / 28.0;
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("ANGLE", currentAngle());
        telemetry.addData("MOTIF", sorter.getCurMotif());
        telemetry.addData("CORRECT MOTIF", sorter.isCorrectMotif());
        telemetry.addData("SHOOTING ALLOWED", isShootable());
        telemetry.addData("CURRENT VELOCITY LEFT", left.getVelocity());
        telemetry.addData("CURRENT RPM LEFT", toRPM(left.getVelocity()));
        telemetry.addData("CURRENT REVOLVER TICKS", revolver.getCurrentPosition());
        telemetry.addData("REVOLVER SPEED TPS", revolver.getVelocity());
    }

    public void snapToNearestSlot() {
        double ticksPerTurn = ShooterConf.SORT_MOTOR_TICKS_PER_TURN;
        double ticksPerSlot = ticksPerTurn / 3.0;
        double offsetTicks = 0; //(60.0 * ticksPerTurn) / 360.0; // ставим в положение для сканирования

        globalTarget = (int) (Math.round((globalTarget - offsetTicks) / ticksPerSlot) * ticksPerSlot + offsetTicks);

        revolver.setTargetPosition(globalTarget);
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        revolver.setVelocity(ShooterConf.REVOLVER_VELOCITY);
    }

    private BallDetection.BallColor getDetectedColor() {
        NormalizedRGBA colors = ((NormalizedColorSensor) cs).getNormalizedColors();
        float[] hsv = new float[3];
        Color.RGBToHSV((int) (colors.red * 255), (int) (colors.green * 255), (int) (colors.blue * 255), hsv);

        telemetry.addData("hue", hsv[0]);
        telemetry.addData("sat", hsv[1]);
        telemetry.addData("val", hsv[2]);

        if (checkColors(hsv, ShooterConf.cslowPurple, ShooterConf.cshighPurple)) {
            return BallDetection.BallColor.PURPLE;
        }
        if (checkColors(hsv, ShooterConf.cslowGreen, ShooterConf.cshighGreen)) {
            return BallDetection.BallColor.GREEN;
        }
        return null;
    }

    private boolean isBallInRange() {
        DistanceSensor sensorDistance = (DistanceSensor) cs;
        double distanceInCm = sensorDistance.getDistance(DistanceUnit.CM);

        return distanceInCm <= ShooterConf.BALL_DETECTION_THRESHOLD;
    }

    public void scanBall() {
        if (sorter.isMotifFull()) return;

        BallDetection.BallColor detectedColor = getDetectedColor();
        boolean ballInRange = isBallInRange();

        boolean currentlySeeingBall = ballInRange && (detectedColor != null);

        if (currentlySeeingBall) {
            if (!potentialBallDetected) {
                potentialBallDetected = true;
                debounceTimer.reset();
            } else if (debounceTimer.milliseconds() >= ShooterConf.SCANNED_BALL_MS && !ballPresent) {
                processNewBall(detectedColor);
                ballPresent = true;
            }
        } else {
            potentialBallDetected = false;
            ballPresent = false;
        }
    }

    private void finalizeMotif() {
        if (sorter.isValid()) {
            sorter.setCorrectMotif(alignRevolverToTarget());
        } else {
            sorter.setCorrectMotif(false);
        }
    }

    public boolean alignRevolverToTarget() {
        if (ShooterConf.TARGET_MOTIF == null || ShooterConf.TARGET_MOTIF.isEmpty()) return false;
        // Default offset is 60 degrees
        double finalRotationDeg = isNearShootingSlot() ? 0 : 60;
        int moveSlots = sorter.getMoveSlots();

        // Add the extra rotation based on the required slots
        if (moveSlots == 1) finalRotationDeg += 120;
        if (moveSlots == 2) finalRotationDeg -= 120; // 60 - 120 = -60 degrees

        // Send ONE single command so the motor uses a single PID curve
        rotateRevolver(finalRotationDeg);

        sorter.setCurMotif(ShooterConf.TARGET_MOTIF);
        return true;
    }



    private void processNewBall(BallDetection.BallColor color) {
        sorter.appendBallToMotif(color);

        if (sorter.isMotifFull()) {
            finalizeMotif();
        } else {
            rotateRevolver(120);
        }
    }

    public void resetRevolverTicks() {
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        revolver.setTargetPosition(0);
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        originalTargetBeforeJam = 0;
        syncTicks();
    }

    public boolean checkColors(float[] hsv, Scalar low, Scalar high) {
        boolean hueMatch;

        if (low.val[0] > high.val[0]) {
            // This handles the Red wrap-around (e.g., low is 350, high is 10)
            hueMatch = (hsv[0] >= low.val[0] || hsv[0] <= high.val[0]);
        } else {
            // Standard range check
            hueMatch = (hsv[0] >= low.val[0] && hsv[0] <= high.val[0]);
        }

        return hueMatch && (hsv[1] >= low.val[1] && hsv[1] <= high.val[1]) && (hsv[2] >= low.val[2] && hsv[2] <= high.val[2]);
    }

    public boolean isNearShootingSlot() {
        double currentAngle = currentAngle() % 360;
        if (currentAngle < 0) currentAngle += 360;

        boolean nearSlot1 = Math.abs(currentAngle - 60) < 8;
        boolean nearSlot2 = Math.abs(currentAngle - 180) < 8;
        boolean nearSlot3 = Math.abs(currentAngle - 300) < 8;

        boolean isNearSlot =  (nearSlot1 || nearSlot2 || nearSlot3);
        return isNearSlot;
    }

    public boolean isShootable() {
        return isNearShootingSlot() && !shot;
    }
    public void pushBall(boolean push) {
        if (push) {
            pusher.setState(Pusher.PusherState.PUSH);
        } else {
            pusher.setState(Pusher.PusherState.NEUTRAL);
        }
    }

    public double currentAngle() {
        return revolver.getCurrentPosition() / ShooterConf.SORT_MOTOR_TICKS_PER_TURN * 360.0;
    }

    public void setPids() {
        revolver.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(ShooterConf.SORT_P, 0, 0, 0));
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(ShooterConf.SHOOTER_P, ShooterConf.SHOOTER_I, ShooterConf.SHOOTER_D, ShooterConf.SHOOTER_F));
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(ShooterConf.SHOOTER_P, ShooterConf.SHOOTER_I, ShooterConf.SHOOTER_D, ShooterConf.SHOOTER_F));
    }


    public void setVelocity(double rpm) {
        left.setVelocity(rpm);
        right.setVelocity(rpm);
    }

    /**
     * Calculates RPM for a given distance using a quadratic regression.
     * <p>
     * Coefficients are sourced from {@link ShooterConf#SHOOTER_A}, {@link ShooterConf#SHOOTER_B}, and {@link ShooterConf#SHOOTER_C}.
     * @param distance distance to AprilTag (meters)
     * @return target shooter RPM
     */
    public double calculateRpmByRegression(double distance) {
        double a = ShooterConf.SHOOTER_A;
        double b = ShooterConf.SHOOTER_B;
        double c = ShooterConf.SHOOTER_C;

        double targetRpm = a * distance * distance + b * distance + c;
        return Math.max(ShooterConf.MIN_SPEED, Math.min(ShooterConf.MAX_SPEED, targetRpm));
    }

    /**
     * Calculates RPM for a given distance using a {@link InterpolationTable}.
     * @param distance distance to AprilTag (meters)
     * @return target shooter RPM
     */
    public double calculateRpmByTable(double distance) {
        return ShooterConf.SHOOTER_TABLE.getInterpolatedValue(distance);
    }

    public boolean shoot() {
        if (state == Shooter.ShooterState.IDLE && isShootable()) {
            pushBall(true);
            shot = true;
            state = ShooterState.SHOOTING;
            shooterTime.reset();
            return true;
        }
        return false;
    }

    private boolean shouldScan() {
        return !isShootable() && !manualControl && !sorter.isMotifFull() && scanAllowed;// && ShooterConf.IS_AUTO;
    }

    public void appendBallToMotif(char color) {
        sorter.appendBallToMotif(color);
    }

    public enum ShooterState {IDLE, SHOOTING, STOP_SHOOTING, REVOLVER_TURNING}
}
