package org.firstinspires.ftc.team28420;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team28420.handlers.Actions;
import org.firstinspires.ftc.team28420.types.AprilTag;
import org.firstinspires.ftc.team28420.types.MovementParams;

@TeleOp(name = "BLUE MAIN TELEOP", group = "MAIN")
public class BlueTeleop extends LinearOpMode {
    private Actions actions;

    private void initialize() {
        telemetry = Actions.getFTCDashboardTelemetry(telemetry);

        actions = new Actions(hardwareMap, AprilTag.BLUE);
        actions.setup();
    }

    private void move() {
        if (gamepad1.left_stick_button) {
            actions.brake();

        } else if (gamepad1.left_bumper) {
            actions.moveToAprilTag();
        } else {
            manualMove();
        }
    }

    private void manualMove() {
        float forward = Actions.getCubic(Actions.withDeathzone(gamepad1.left_stick_y, 0.1f));
        float strafe = Actions.getCubic(Actions.withDeathzone(gamepad1.left_stick_x, 0.1f));
        float turn = Actions.getCubic(Actions.withDeathzone(gamepad1.right_stick_x, 0.1f));

        actions.moveByParamsWithOffset(new MovementParams(forward, strafe, turn), Math.PI / 2);
    }

    private void shoot() {
        if (gamepad1.left_bumper && gamepad1.right_trigger_pressed) {
            actions.shoot();
        } else if (gamepad1.left_bumper) {
            actions.shootWithCheck();
        }
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            move();
            actions.setDribblerState(gamepad1.left_trigger_pressed, gamepad1.left_bumper);
            shoot();

            actions.update();
            actions.log(telemetry);
            telemetry.update();
        }
    }
}
