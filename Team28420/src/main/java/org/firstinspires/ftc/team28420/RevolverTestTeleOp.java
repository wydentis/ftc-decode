package org.firstinspires.ftc.team28420;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team28420.module.Revolver;
import org.firstinspires.inspection.GamepadInspection;

import java.util.List;

@TeleOp
public class RevolverTestTeleOp extends LinearOpMode {
    Revolver revolver;

    double angle = 60;
    boolean pressed = false;

    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        revolver = new Revolver(hardwareMap);
        revolver.setup();
        waitForStart();

        while(opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if(gamepad1.right_bumper && !pressed) {
                revolver.rotateRevolver(angle);
                pressed = true;
            }
            if(gamepad1.left_bumper && !pressed) {
                revolver.rotateRevolver(-angle);
                pressed = true;
            }

            if(gamepad1.dpad_up && !pressed) {
                angle += 60;
                pressed = true;
            }else if(gamepad1.dpad_down && !pressed) {
                angle -= 60;
                pressed = true;
            }

            pressed = gamepad1.right_bumper || gamepad1.left_bumper
                    || gamepad1.dpad_up || gamepad1.dpad_down;

            revolver.update();
            revolver.log(telemetry);
            telemetry.addData("current turn angle", angle);
            telemetry.update();
        }
    }

}
