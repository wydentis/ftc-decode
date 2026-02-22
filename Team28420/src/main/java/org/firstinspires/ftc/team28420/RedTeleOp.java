package org.firstinspires.ftc.team28420;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team28420.module.Actions;
import org.firstinspires.ftc.team28420.module.Camera;
import org.firstinspires.ftc.team28420.module.Movement;
import org.firstinspires.ftc.team28420.module.Shooter;
import org.firstinspires.ftc.team28420.types.AprilTag;
import org.firstinspires.ftc.team28420.util.Config;

@TeleOp(name = "RED MAIN", group = "New Actions")
public class RedTeleOp extends LinearOpMode {
    private boolean dpad_active = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Config.Etc.telemetry = telemetry;

        Actions act = new Actions(
                new Movement(
                        hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.LEFT_TOP_MOTOR),
                        hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.RIGHT_TOP_MOTOR),
                        hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.LEFT_BOTTOM_MOTOR),
                        hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.RIGHT_BOTTOM_MOTOR)
                ),
                hardwareMap.get(IMU.class, Config.GyroConf.IMU),
                new Camera(hardwareMap.get(WebcamName.class, Config.CameraConf.WEBCAM)),
                new Shooter(hardwareMap),
                hardwareMap.get(Servo.class, "parkingServo")
        );

        act.init();

        waitForStart();

        act.afterStart();
//        act.updateHeading();
        Config.ShooterConf.TARGET_MOTIF = null;
        while (opModeIsActive()) {
            if (Config.ShooterConf.TARGET_MOTIF == null) {
                act.setMotif();
            }

            telemetry.addData("scanned motif", Config.ShooterConf.TARGET_MOTIF);

            if (gamepad1.right_trigger > 0.2) {
//                act.updateHeading();
                act.move(act.getRatiosForApriltag(AprilTag.RED, 2, Config.CameraConf.RANGE_TO_TAG));
            }
            else if (gamepad1.right_bumper) {
                act.move(act.getRatiosLookApriltag(AprilTag.RED, 0, Config.CameraConf.RANGE_TO_TAG));
            }
            else {
                act.move(
                        act.getRatios(
                                act.getCubic(act.withDeathzone(gamepad1.left_stick_x, Config.GamepadConf.LEFT_DEAD_ZONE)),
                                -1 * act.getCubic(act.withDeathzone(gamepad1.left_stick_y, Config.GamepadConf.LEFT_DEAD_ZONE)),
                                act.getCubic(act.withDeathzone(gamepad1.right_stick_x, Config.GamepadConf.RIGHT_DEAD_ZONE))
                        )
                );
            }

            if (gamepad2.triangle || gamepad2.circle) {
                if (gamepad2.circle) {
                    act.resetRevolverTicks();
                }
                act.toggleShooterManualControl(false);

                gamepad2.setLedColor(0, 255, 0, -1);
                gamepad2.rumble(0);
            }

            int rovation = 0;

            if (gamepad2.dpad_left)
                rovation = -60;
            else if (gamepad2.dpad_right)
                rovation = 60;
            else if (gamepad2.dpad_up)
                rovation = -2;
            else if (gamepad2.dpad_down)
                rovation = 2;

            if (rovation != 0) {
                if (!dpad_active) {
                    act.revolverRotate(rovation);

                    gamepad2.setLedColor(255, 0, 0, -1);
                    gamepad2.rumble(-1);

                    dpad_active = true;
                }
            } else {
                dpad_active = false;
            }

            if (gamepad1.left_bumper) {
                double coef = (gamepad1.left_trigger > 0.5) ? -0.5 : 1.0;
                act.setDribblerVelocityCoefficient((float) coef);
            } else {
                act.setDribblerVelocityCoefficient(0);
            }

            if (gamepad2.right_trigger > 0.4) {
                act.setShooterVelocityCoefficient(gamepad2.right_trigger * gamepad2.right_trigger);
            } else act.setShooterVelocityCoefficient(0);

            if (gamepad2.right_bumper){
                act.shoot();
            }

            act.updateShooter();

            if (gamepad1.dpad_up) {
                act.park();
            }

            act.log();
            telemetry.update();
        }
    }
}
