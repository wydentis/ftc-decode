 package org.firstinspires.ftc.team28420;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team28420.config.CameraConf;
import org.firstinspires.ftc.team28420.config.GamepadConf;
import org.firstinspires.ftc.team28420.config.ShooterConf;
import org.firstinspires.ftc.team28420.module.Actions;
import org.firstinspires.ftc.team28420.types.AprilTag;
import org.firstinspires.ftc.team28420.types.MovementParams;
import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.Position;
import org.firstinspires.ftc.team28420.types.WheelsRatio;

 @TeleOp(name = "teleop main BLUE", group = "MAIN")
public class BlueTeleop extends LinearOpMode {
    private Actions act;
    private boolean dpadPressed = false;
    private void initialize() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        act = new Actions(hardwareMap, false, telemetry);

        ShooterConf.IS_AUTO = false;

        act.init();
    }
    private void handleTargeting() {
        if (ShooterConf.TARGET_MOTIF == null) {
            act.setMotif();
        }
        telemetry.addData("scanned motif", ShooterConf.TARGET_MOTIF);
    }
    private void handleMovement() {
        if (gamepad1.right_trigger > 0.2) {
            act.camIdle();
            act.move(act.getRatiosForApriltag(AprilTag.BLUE, 0.07, CameraConf.RANGE_TO_TAG));
        } else if (gamepad1.right_bumper) {
            act.camIdle();
            act.move(act.getRatiosLookApriltag(AprilTag.BLUE, 0, CameraConf.RANGE_TO_TAG));
        } else {
            manualDrive();
        }
    }
     private void manualDrive() {
         if (gamepad1.left_stick_button) {
             act.brake();
             return;
         }

         double x = act.getCubic(act.withDeathzone(gamepad1.left_stick_x, GamepadConf.LEFT_DEAD_ZONE));
         double y = -act.getCubic(act.withDeathzone(gamepad1.left_stick_y, GamepadConf.LEFT_DEAD_ZONE));
         double rx = act.getCubic(act.withDeathzone(gamepad1.right_stick_x, GamepadConf.RIGHT_DEAD_ZONE));

         WheelsRatio<Double> ratios = act.getRatios(new MovementParams(
                 new PolarVector(new Position(x, y)), rx));
         act.move(ratios);
     }
    private void indicateReady() {
        gamepad2.setLedColor(0, 255, 0, -1);
        gamepad2.rumble(0);
    }
    private void handleShooter() {
        if (gamepad2.triangle || gamepad2.circle) {
            if (gamepad2.circle) act.resetRevolverTicks();
            act.toggleShooterManualControl(false);
            indicateReady();
        }
        handleRevolverInput();

        float distanceToAprilTag = act.getDistanceToAprilTag(AprilTag.BLUE);
        act.prepareForShoot(distanceToAprilTag);
        if (gamepad2.right_bumper) act.shoot();
    }
    private void rotateRevolver(int degrees) {
        act.revolverRotate(degrees);
        gamepad2.setLedColor(255, 0, 0, -1);
        gamepad2.rumble(-1);
    }
    private void handleRevolverInput() {
        if(gamepad2.square) act.setScanAllowed(true);
        if(gamepad2.cross) act.setScanAllowed(false);

        if (!dpadPressed) {
            int shooterMultiplier = gamepad2.right_trigger > 0.2?2:1;

            if (gamepad2.dpad_left) rotateRevolver(-60 * shooterMultiplier);
            if (gamepad2.dpad_right) rotateRevolver(60 * shooterMultiplier);
        }
        dpadPressed = (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down);

        if (gamepad2.dpad_up) rotateRevolver(-5);
        if (gamepad2.dpad_down) rotateRevolver(5);
    }
    private void handleIntakeAndParking() {
        if (gamepad1.left_bumper) {
            float power = (gamepad1.left_trigger > 0.5) ? -0.5f : 1.0f;
            act.setDribblerVelocityCoefficient(power);
        } else {
            if(gamepad2.right_trigger < 0.4)
                act.setDribblerVelocityCoefficient(0);
        }
        if (gamepad1.dpad_up) act.park();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        act.afterStart();
        ShooterConf.TARGET_MOTIF = null;
        while (opModeIsActive()) {
            act.updateLastAngles();
            act.updateApriltags();
            handleTargeting();
            handleMovement();
            handleShooter();
            handleIntakeAndParking();
            act.updateShooter();
            act.log();
            act.clearCache();
            telemetry.update();
        }
    }
}