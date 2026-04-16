package org.firstinspires.ftc.team28420.handlers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team28420.config.ShooterConf;
import org.firstinspires.ftc.team28420.module.Pusher;
import org.firstinspires.ftc.team28420.module.Revolver;
import org.firstinspires.ftc.team28420.module.ScannerSorter;

/**
 * Shooter class
 * is responsible for scanning balls, sorting motif with revolver and flywheel activation
 */
@Config
public class IntakeHandler {

    /*** HARDWARE ***/
    private final Pusher pusher;
    private ScannerSorter scannerSorter = null;
    private Revolver revolver = null;


    /*** TIMERS ***/
    private final ElapsedTime shooterTime = new ElapsedTime();

    private static double pusherReadyTime = 175;

    private ShooterState state = ShooterState.IDLE;
    public IntakeHandler(HardwareMap hMap, Telemetry telemetry) {
        pusher = new Pusher(hMap);
        revolver = new Revolver(hMap);
        scannerSorter = new ScannerSorter(hMap, () -> revolver.rotateRevolver(120), this::alignRevolverToTarget);
    }

    public void setup() {
        pusher.setup();
        revolver.setup();
    }

    public void update() {
        revolver.update();

        switch (state) {
            case SHOOTING_PREPARE:
                if(shooterTime.milliseconds() > pusherReadyTime) {
                    state = ShooterState.SHOOTING_SPIN;
                    revolver.rotateRevolver(scannerSorter.getMotif().length() * 120.0 + 60);
                }
                break;
            case SHOOTING_SPIN:
                if(!revolver.isBusy()) {
                    state = ShooterState.STOP_SHOOTING;
                    pushBall(false);

                    shooterTime.reset();
                }
                break;
            case STOP_SHOOTING:
                if(shooterTime.milliseconds() > pusherReadyTime) {
                    snapToNearestSlot();
                    state = ShooterState.IDLE;
                    shooterTime.reset();
                }
                break;
            case IDLE:
                if(!isNearShootingSlot()) scannerSorter.scanBall();
                break;
        }
    }
    /**
     * @return True if revolver is in shooting position
     */
    private boolean isNearShootingSlot() {
        double currentAngle = revolver.currentAngle() % 360;
        if (currentAngle < 0) currentAngle += 360;

        boolean nearSlot1 = Math.abs(currentAngle - 60) < 8;
        boolean nearSlot2 = Math.abs(currentAngle - 180) < 8;
        boolean nearSlot3 = Math.abs(currentAngle - 300) < 8;

        return (nearSlot1 || nearSlot2 || nearSlot3);
    }

    private boolean alignRevolverToTarget() {
        double finalRotationDeg = isNearShootingSlot() ? 0 : 60;
        int moveSlots = scannerSorter.getMoveSlots();

        // Add the extra rotation based on the required slots
        if (moveSlots == 1) finalRotationDeg += 120;
        if (moveSlots == 2) finalRotationDeg -= 120; // 60 - 120 = -60 degrees

        revolver.rotateRevolver(finalRotationDeg);

        scannerSorter.setCurMotif(ShooterConf.TARGET_MOTIF);
        return true;
    }

    /**
     * Snaps revolver to the nearest slot where ball intake is possible.
     */
    private void snapToNearestSlot() {
        double degPerSlot = 120.0;
        double offsetDeg = 0;

        double currentPosDeg = revolver.currentAngle();
        double targetPosDeg = Math.round((currentPosDeg - offsetDeg) / degPerSlot) * degPerSlot + offsetDeg;
        double deltaDeg = targetPosDeg - currentPosDeg;

        revolver.rotateRevolver(deltaDeg);
    }

    // TODO: DELETE IF NOT USED AFTER WELL-TESTING
//
//    public void toggleManualControl(boolean active) {
//        if (state == ShooterState.IDLE) {
//            if (!active && manualControl) {
//                snapToNearestSlot();
//            }
//            if (active != manualControl) {
//                manualControl = active;
//                sorter.resetMotif();
//            }
//        }
//    }

    public void log(Telemetry telemetry) {
        scannerSorter.log(telemetry);
    }

    private void pushBall(boolean push) {
        if (push) {
            pusher.setState(Pusher.PusherState.PUSH);
        } else {
            pusher.setState(Pusher.PusherState.NEUTRAL);
        }
    }

    public boolean shoot() {
        if (state == ShooterState.IDLE && !revolver.isBusy()) {
            pushBall(true);
            state = ShooterState.SHOOTING_PREPARE;
            shooterTime.reset();
            return true;
        }
        return false;
    }

    public enum ShooterState {IDLE, SHOOTING_PREPARE, SHOOTING_SPIN, STOP_SHOOTING}
}
