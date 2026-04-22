package org.firstinspires.ftc.team28420;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team28420.handlers.Actions;
import org.firstinspires.ftc.team28420.types.AprilTag;
import org.firstinspires.ftc.team28420.types.Position;

@Autonomous(name = "BLUE MAIN AUTO", group = "MAIN")
public class BlueAutonomous extends LinearOpMode {

    private Actions actions;

    private void initialize() {
        telemetry = Actions.getFTCDashboardTelemetry(telemetry);

        actions = new Actions(hardwareMap, AprilTag.BLUE);
        actions.setup();
    }

    @Override
    public void runOpMode() {
        initialize();

        ElapsedTime elapsedTime = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {
            if (elapsedTime.milliseconds() <= 10000) {
                actions.moveToPosition(new Position(-200, -200), 0);
            }

            actions.update();
            actions.log(telemetry);
            telemetry.update();
        }
    }

}
