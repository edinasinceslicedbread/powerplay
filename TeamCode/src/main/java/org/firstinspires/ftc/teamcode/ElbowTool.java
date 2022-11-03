package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElbowTool {
    public MotorEx elbow;
    public LinearOpMode opmode;

    public void init(HardwareMap hwMap, LinearOpMode opmode) {
        elbow = new MotorEx(hwMap, "elbow");
        this.opmode = opmode;

        elbow.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        elbow.setRunMode(MotorEx.RunMode.PositionControl);
        elbow.resetEncoder();
        elbow.setTargetPosition(0);

        // Initialize the PID variables
        elbow.setPositionCoefficient(0.05);

        // Reset the encoders and set tolerance
        elbow.set(0);
        elbow.setPositionTolerance(13.6);

    }

    public void moveAbsolute(int target) {

        // Set some guardrails
        target = MathUtils.clamp(target, 0, 930);

        elbow.setTargetPosition(target);

        // Move at 75% power until position is reached, braking will start before it reaches the position
        while (!elbow.atTargetPosition() && opmode.opModeIsActive()) {
            elbow.set(.1);
        }

        elbow.stopMotor();
    }

    public void moveMaxRange() {
        moveAbsolute(930);
    }

    public void moveMinRange() {
        moveAbsolute(0);
    }

    public int getCurrentPosition() {
        return elbow.getCurrentPosition();
    }

    public void setTargetPosition(int targetPosition) {
        elbow.setTargetPosition(targetPosition);
    }

    public void stopMotor() {
        elbow.stopMotor();
    }

    public void set(double maxPower) {
        elbow.set(maxPower);
    }
}

