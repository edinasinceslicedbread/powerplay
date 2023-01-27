package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualLiftTool {
    public MotorEx liftFront, liftRear;
    public MotorGroup lift;

    public void init(HardwareMap hwMap) {
        liftFront = new MotorEx(hwMap, "lift");
        liftFront.setInverted(true);

        liftRear = new MotorEx(hwMap, "liftRear");
        liftRear.setInverted(false);

        lift = new MotorGroup(liftRear, liftFront);

        lift.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        lift.setRunMode(MotorEx.RunMode.PositionControl);
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

        lift.set(feedforward.calculate(.1,.05));

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

