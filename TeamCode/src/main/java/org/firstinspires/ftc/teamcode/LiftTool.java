package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.MathUtils;

public class LiftTool {
    public MotorEx lift;
    public LinearOpMode opmode;

    public void init(HardwareMap hwMap, LinearOpMode opmode) {
        lift = new MotorEx(hwMap, "lift");
        this.opmode = opmode;

        lift.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        lift.setRunMode(MotorEx.RunMode.PositionControl);
        lift.setInverted(true);
        lift.resetEncoder();
        lift.setTargetPosition(0);

        // Initialize the PID variables
        lift.setPositionCoefficient(0.05);

        // Reset the encoders and set tolerance
        lift.set(0);
        lift.setPositionTolerance(13.6);

    }

    public void moveAbsolute(int target) {

        // Set some guardrails
        target = MathUtils.clamp(target, 0, 3000);

        lift.setTargetPosition(target);

        // Move at 75% power until position is reached, braking will start before it reaches the position
        while (!lift.atTargetPosition() && opmode.opModeIsActive()) {
            lift.set(.2);
        }

        lift.stopMotor();
    }

    public void moveMaxRange() {
        moveAbsolute(3000);
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

