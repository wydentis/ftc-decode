package org.firstinspires.ftc.team28420.module;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Pusher {
    public enum PusherState {
        INIT,
        PUSH,
        NEUTRAL
    }
    public static double INITPOS = 0.1;
    public static double PUSHPOS = 0.54;
    public static double NEUTRALPOS = 0.1;
    private final Servo pusher;
    private PusherState state = PusherState.INIT;

    public Pusher(HardwareMap hMap) {
        this.pusher = hMap.get(Servo.class, "pusher");
    }

    public void setup() {
        setState(PusherState.NEUTRAL);
    }
    public void setState(PusherState state) {
        this.state = state;
        updatePosition();
    }

    private void updatePosition() {
        if(state == PusherState.PUSH) pusher.setPosition(PUSHPOS);
        else if(state == PusherState.NEUTRAL) pusher.setPosition(NEUTRALPOS);
        else pusher.setPosition(INITPOS);
    }
}
