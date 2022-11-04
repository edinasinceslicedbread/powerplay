package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristTool {
    public Servo wrist;
    public LinearOpMode opmode;
    public Telemetry telemetry;

    public void init(HardwareMap hwMap) {
        wrist=hwMap.get(Servo.class, "wrist");
    }

    public void moveAbsolute(double target) {

        if (target>1) {
            target = 1;
        } else if (target<0) {
            target = 0;
        }

        wrist.setPosition(target);
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


