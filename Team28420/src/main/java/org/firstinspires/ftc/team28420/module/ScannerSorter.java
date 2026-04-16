package org.firstinspires.ftc.team28420.module;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team28420.processors.BallDetection;

public class ScannerSorter {
    /*** SCANNER CONSTANTS ***/
    public static double SCANNED_BALL_MS = 30;
    public static double BALL_DETECTION_THRESHOLD = 2;
    private boolean scanAllowed = true;
    private boolean ballPresent = false;
    private boolean potentialBallDetected = false;
    private final MotifSorter sorter = new MotifSorter();

    /*** SORTER CONSTANTS ***/
    public static String TARGET_MOTIF = null;

    /*** HARDWARE ***/
    private final ColorSensor cs;

    /*** TIMERS ***/
    private final ElapsedTime debounceTimer = new ElapsedTime();

    /*** CALLBACKS ***/
    private Runnable onBallDetected = null; // action which happens after ball detected (IF NOT FULL)
    private Runnable onMotifGathered = null; // action when motif is full

    public ScannerSorter(HardwareMap hMap, Runnable onBallDetected, Runnable onMotifFull) {
        this.cs = hMap.get(ColorSensor.class, "colorSensor");

        this.onBallDetected = onBallDetected;
        this.onMotifGathered = onMotifFull;
    }

    /**
     * HARD CONTROL ON SCANNING
     * @param allowed
     */
    public void setScanAllowed(boolean allowed) {
        scanAllowed = allowed;
    }

    /**
     * Scans the ball and if detected appends ball to current motif and if its full then calling onMotifFull
     *
     */
    public void scanBall() {
        if (!scanAllowed || sorter.isMotifFull()) return;

        BallDetection.BallColor detectedColor = getDetectedColor();
        boolean ballInRange = isBallInRange();

        boolean currentlySeeingBall = ballInRange && (detectedColor != null);

        if (currentlySeeingBall) {
            if (!potentialBallDetected) {
                potentialBallDetected = true;
                debounceTimer.reset();
            } else if (debounceTimer.milliseconds() >= SCANNED_BALL_MS && !ballPresent) {
                processNewBall(detectedColor);
                ballPresent = true;
            }
        } else {
            potentialBallDetected = false;
            ballPresent = false;
        }
    }

    public String getMotif() {
        return sorter.getCurMotif();
    }

    private BallDetection.BallColor getDetectedColor() {
        NormalizedRGBA colors = ((NormalizedColorSensor) cs).getNormalizedColors();

        // mostly green
        if (colors.green > colors.red * 1.5 && colors.green > colors.blue * 1.5) {
            return BallDetection.BallColor.GREEN;
        }
        // red + blue but not green
        if (colors.blue > colors.green * 1.2 && colors.red > colors.green) {
            return BallDetection.BallColor.PURPLE;
        }
        return null;
    }
    private boolean isBallInRange() {
        DistanceSensor sensorDistance = (DistanceSensor) cs;
        double distanceInCm = sensorDistance.getDistance(DistanceUnit.CM);

        return distanceInCm <= BALL_DETECTION_THRESHOLD;
    }

    private void processNewBall(BallDetection.BallColor color) {
        sorter.appendBallToMotif(color);

        if (sorter.isMotifFull()) {
            if(sorter.isCorrectMotif() && onMotifGathered != null) onMotifGathered.run();
        } else {
            if(onBallDetected != null) onBallDetected.run();
        }
    }

    public int getMoveSlots() {
        return sorter.getMoveSlots();
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("MOTIF", sorter.getCurMotif());
        telemetry.addData("CORRECT MOTIF", sorter.isCorrectMotif());
    }

    /**
     * sets motif manually
     * @param motif
     */
    public void setCurMotif(String motif) {
        sorter.setCurMotif(motif);
    }
}
