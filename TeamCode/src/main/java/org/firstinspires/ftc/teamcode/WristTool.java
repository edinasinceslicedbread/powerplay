package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristTool {
    public Servo wrist;
    public Telemetry telemetry;

    public void init(HardwareMap hwMap) {
        wrist=hwMap.get(Servo.class, "wrist");
    }

    public void moveAbsolute(double target) {
        wrist.setPosition(target);
    }

    public double getPosition() {
        return wrist.getPosition();
    }

    public void moveMaxRange() {
        moveAbsolute(1);
    }

    public void moveMinRange() {
        moveAbsolute(0);
    }

    public void reset() {
        moveMinRange();
    }
}


