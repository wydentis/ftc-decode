package org.firstinspires.ftc.team28420.handlers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team28420.module.Webcam;
import org.firstinspires.ftc.team28420.processors.AprilTagWrapper;
import org.firstinspires.ftc.team28420.types.AprilTag;
import org.firstinspires.ftc.team28420.types.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class CameraHandler {

    private final Webcam webcam;
    private final AprilTagWrapper aprilTagWrapper;

    public CameraHandler(HardwareMap hMap) {
        this.aprilTagWrapper = new AprilTagWrapper();
        webcam = new Webcam(hMap, aprilTagWrapper.getProcessor());
    }

    public void setup() {
        webcam.setup();
        aprilTagWrapper.setup();
    }

    public void update() {
        aprilTagWrapper.update();
    }

    public Position getRobotPosition(AprilTag aprilTag) {
        AprilTagDetection detection = aprilTagWrapper.getDetection(aprilTag);
        if (detection == null) {
            return null;
        }
        return new Position(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y);
    }

    public Double getAprilTagBearing(AprilTag aprilTag) {
        AprilTagDetection detection = aprilTagWrapper.getDetection(aprilTag);
        if (detection == null) {
            return null;
        }
        return detection.ftcPose.bearing;
    }

    public String getMotif() {
        AprilTagDetection detection = aprilTagWrapper.getDetection(AprilTag.GREEN);
        if (detection == null) {
            return null;
        }
        return AprilTag.getMotif(detection.id);
    }

    public void log(Telemetry telemetry) {
        aprilTagWrapper.log(telemetry);
    }
}
