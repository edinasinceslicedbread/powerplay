package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeTool {
    public Servo intake;
    public Telemetry telemetry;

    public void init(HardwareMap hwMap) {
        intake=hwMap.get(Servo.class, "intake");
    }

    public void moveAbsolute(double target) {

        if (target>1) {
            target = 1;
        } else if (target<0) {
            target = 0;
        }

        intake.setPosition(target);
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


