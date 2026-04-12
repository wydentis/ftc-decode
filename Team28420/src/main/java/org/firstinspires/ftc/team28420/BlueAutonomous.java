package org.firstinspires.ftc.team28420;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team28420.config.CameraConf;
import org.firstinspires.ftc.team28420.config.ShooterConf;
import org.firstinspires.ftc.team28420.module.Actions;
import org.firstinspires.ftc.team28420.types.AprilTag;

@Autonomous(name = "auto main BLUE", group = "MAIN")
public class BlueAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Actions act = new Actions(hardwareMap, true, telemetry);

        ElapsedTime elapsedTime = new ElapsedTime();
        int counter = 1;
        boolean shootingDone = false;

        act.init();
        act.setDefaultAutoMotif("PPG");
        act.setScanAllowed(false);
        act.resetYaw();

        telemetry.addLine("=== INIT READY ===");
        telemetry.update();

        waitForStart();

        act.afterStart();
        elapsedTime.reset();
        ShooterConf.TARGET_MOTIF = null;
        while (opModeIsActive()) {

            telemetry.addData("scanned motif", ShooterConf.TARGET_MOTIF);
            telemetry.addData("elapsed time", elapsedTime.milliseconds());

            if (elapsedTime.milliseconds() <= 1500) {
                act.move(act.getRatios(0.1, -0.6, 0.1));
            } else if (elapsedTime.milliseconds() <= 2200 && ShooterConf.TARGET_MOTIF == null) {
                act.move(act.getRatios(0, 0, 0));
                act.setMotif();
                act.alignRevolverToTarget();
            } else if (elapsedTime.milliseconds() <= 2700) {
                if (ShooterConf.TARGET_MOTIF == null) {
                    ShooterConf.TARGET_MOTIF = "PGP";
                    act.alignRevolverToTarget();
                }
                act.move(act.getRatios(0, 0, act.getForceToGyro(Math.PI / 4)));
            } else if (elapsedTime.milliseconds() <= 5500) {
                act.move(act.getRatiosForApriltag(AprilTag.BLUE, 0, CameraConf.RANGE_TO_TAG));
            } else if (elapsedTime.milliseconds() <= 9000) {
                act.move(act.getRatios(0, 0, 0));
            } else if (elapsedTime.milliseconds() <= 12000) {
                act.move(act.getRatiosForApriltag(AprilTag.BLUE, 0.32, CameraConf.RANGE_TO_TAG - 0.1));
            } else if (elapsedTime.milliseconds() <= 14000) {
                act.move(act.getRatios(0, 0, act.getForceToGyro(Math.PI / 2)));
            }
//            } else if (elapsedTime.milliseconds() <= 13500) {
//                act.move(act.getRatiosForApriltag(AprilTag.BLUE, 0.30, CameraConf.RANGE_TO_TAG - 0.1));
//            } else if (elapsedTime.milliseconds() <= 16000) {
//                act.move(act.getRatios(0, 0, act.getForceToGyro(-Math.PI / 2)));
//            } else if (elapsedTime.milliseconds() <= 18000) {
//                act.camPeek();
//                act.setScanAllowed(true);
//                act.aimAndDriveToBall();
//            } else if (elapsedTime.milliseconds() <= 20500) {
//                act.camIdle();
//                act.setScanAllowed(false);
//                act.move(act.getRatios(0, -0.5, 0));
//            } else if (elapsedTime.milliseconds() <= 21500) {
//                act.move(act.getRatios(0, 0, act.getForceToGyro(-Math.PI / 5)));
//            } else if (elapsedTime.milliseconds() <= 23500) {
//                act.move(act.getRatiosLookApriltag(AprilTag.BLUE, 0.05, 0));
//            } else if (elapsedTime.milliseconds() <= 25500){
//                act.move(act.getRatios(0, 0, 0));
//            } else if (elapsedTime.milliseconds() <= 26000) {
//                act.move(act.getRatios(0.1, 0.5, 0));
//            } else {
//                act.move(act.getRatios(0, 0, 0));
//            }

            if (elapsedTime.milliseconds() >= 4900 && elapsedTime.milliseconds() <= 7000) {
                act.prepareForShoot(1000);
            } else if (elapsedTime.milliseconds() >= 5550 && elapsedTime.milliseconds() < 23500 && !shootingDone) {
                if (act.isShootable()) {
                    if (act.shoot()) {
                        counter++;
                    }
                }
                if (counter > 3) {
                    shootingDone = true;
                    act.prepareForShoot(0);
                }
            } else if (elapsedTime.milliseconds() >= 23500 && elapsedTime.milliseconds() <= 25000) {
                act.prepareForShoot(1000);
                shootingDone = false;
                counter = 1;
            } else if(elapsedTime.milliseconds() >= 25000 && !shootingDone) {
                if (act.isShootable()) {
                    if (act.shoot()) {
                        counter++;
                    }
                }
                if (counter > 3) {
                    shootingDone = true;
                    act.prepareForShoot(0);
                }
            }


            telemetry.addData("counter", counter);
            act.updateShooter();
            act.updateApriltags();
            act.updateLastAngles();
            act.log();
            telemetry.update();
        }
    }
}
