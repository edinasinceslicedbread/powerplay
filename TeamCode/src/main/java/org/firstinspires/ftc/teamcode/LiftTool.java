package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.MathUtils;

public class LiftTool {
    public MotorEx lift;

    public void init(HardwareMap hwMap) {
        lift = new MotorEx(hwMap, "lift");

        lift.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        lift.setRunMode(MotorEx.RunMode.PositionControl);
        lift.setInverted(true);
        lift.resetEncoder();
        lift.setTargetPosition(0);

        // Initialize the PID variables
        lift.setPositionCoefficient(0.02);
        lift.setPositionTolerance(13.6);

        lift.set(0);

    }

    public void moveAbsolute(int target) {

        // Set some guardrails
        target = MathUtils.clamp(target, 0, 2900);

        lift.setTargetPosition(target);

        ElevatorFeedforward feedforward = new ElevatorFeedforward(
                .2, .2, .1, 0
        );

        lift.set(feedforward.calculate(.01,.01));

    }

    public void moveMaxRange() {
        moveAbsolute(2900);
    }

    public void moveMinRange() {
        moveAbsolute(0);
    }

    public int getCurrentPosition() {
        return lift.getCurrentPosition();
    }

    public void setTargetPosition(int targetPosition) {
        lift.setTargetPosition(targetPosition);
    }

    public void stopMotor() {
        lift.stopMotor();
    }

    public void set(double maxPower) {
        lift.set(maxPower);
    }
}

