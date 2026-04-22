package org.firstinspires.ftc.team28420.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dribbler {

    @Config
    public static class DribblerConf {
        public static int INTAKE_VELOCITY = 5067;
        public static int DROP_VELOCITY = (int) (INTAKE_VELOCITY * 0.3);
    }

    private final DcMotorEx dribblerMotor;

    public Dribbler(HardwareMap hMap) {
        dribblerMotor = hMap.get(DcMotorEx.class, "dribbler");
    }

    public void setDribblerState(DribblerState state) {
        switch (state) {
            case INTAKE:
                dribblerMotor.setVelocity(DribblerConf.INTAKE_VELOCITY);
                break;
            case DROP:
                dribblerMotor.setVelocity(DribblerConf.DROP_VELOCITY);
                break;
            case IDLE:
                dribblerMotor.setVelocity(0);
                break;
        }
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("dribbler velocity", dribblerMotor.getVelocity());
    }

    public enum DribblerState {INTAKE, DROP, IDLE}
}
