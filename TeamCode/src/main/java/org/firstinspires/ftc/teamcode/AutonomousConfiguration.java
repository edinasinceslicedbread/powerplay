package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * Modified: 10/31/2022
 * <p>
 * This class provides configuration for an autonomous opMode.
 * Most games benefit from autonomous opModes that can implement
 * different behavior based on an alliance strategy agreed upon
 * for a specific match.
 * </p>
 * <p>
 * Creating multiple opModes to meet this requirement results in duplicate
 * code and an environment that makes it too easy for a driver to
 * choose the wrong opMode "in the heat of battle."
 * </p>
 * <p>
 * This class is a way to solve these problems.
 * It is designed to used from opMode (iterative) class.
 * The selected options can also be saved to a file, allowing the
 * configuration options to be set before a match and to be available
 * to any op Mode.
 * </p>
 */

public class AutonomousConfiguration {
    private AutonomousOptions autonomousOptions;
    private GamepadEx gamepadEx;
    private Context context;
    private boolean readyToStart;
    private boolean savedToFile;
    private Telemetry telemetry;
    private Telemetry.Item teleStartPosition;
    private Telemetry.Item teleDropLocation;
    private Telemetry.Item teleFirstDrop;
    private Telemetry.Item teleDelayStartSeconds;
    private Telemetry.Item teleReadyToStart;
    private Telemetry.Item teleSavedToFile;

    /*
     * Pass in the gamepad and telemetry from your opMode.
     */
    public void init(Gamepad gamepad, Telemetry telemetry1, Context context) {
        this.context = context;
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        gamepadEx = new GamepadEx(gamepad);
        this.telemetry = telemetry1;
        // See if we saved the options yet. If not, save the defaults.
        autonomousOptions = new AutonomousOptions();
        if (!readWriteAutoOptions.optionsAreSaved()) {
            resetOptions();
            this.SaveOptions();
        } else {
            autonomousOptions = getSaveAutoOptions();
        }
        autonomousOptions.setFirstDrop(AutonomousOptions.FirstDrop.C2);
        autonomousOptions.setDropLocation(AutonomousOptions.DropLocation.D3);

        ShowHelp();
    }

    public AutonomousOptions.StartPosition getStartPosition() {
        return autonomousOptions.getStartPosition();
    }

    public AutonomousOptions.DropLocation getDropLocation() {
        return autonomousOptions.getDropLocation();
    }

    public AutonomousOptions.FirstDrop getFirstDrop() {
        return autonomousOptions.getFirstDrop();
    }

    public int getDelayStartSeconds() {
        return autonomousOptions.getDelayStartSeconds();
    }

    public boolean getReadyToStart() {
        return readyToStart;
    }

    private void ShowHelp() {
        teleStartPosition = telemetry.addData("D-pad left/right, select start position", autonomousOptions.getStartPosition());
        teleFirstDrop = telemetry.addData("D-pad down to cycle initial drop", autonomousOptions.getFirstDrop());
        teleDropLocation = telemetry.addData("D-pad up to cycle drop location", autonomousOptions.getDropLocation());
        teleDelayStartSeconds = telemetry.addData("Left & Right buttons, Delay Start", autonomousOptions.getDelayStartSeconds());
        teleReadyToStart = telemetry.addData("Ready to start: ", getReadyToStart());
        teleSavedToFile = telemetry.addData("Saved to file:", savedToFile);
        telemetry.addLine("Back button resets all options.");
    }

    // Call this in the init_loop from your opMode. It will returns true if you press the
    // game pad Start.
    public boolean init_loop() {

        boolean doneFlag = false;

        gamepadEx.readButtons();

        //Set default options (ignore what was saved to the file.)
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.BACK)) {
            resetOptions();
        }

        //Start Position
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
            autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.Right);
            telemetry.speak("start right");
        }

        if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.Left);
            telemetry.speak("start left");
        }
        teleStartPosition.setValue(autonomousOptions.getStartPosition());

        //Drop Location
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            AutonomousOptions.DropLocation dropLocation = autonomousOptions.getDropLocation().getNext();
            switch (dropLocation) {
                case B2:
                    telemetry.speak("Drop on B2");
                    break;
                case B3:
                    telemetry.speak("Drop on B3");
                    break;
                case D2:
                    telemetry.speak("Drop on D2");
                    break;
                case D3:
                    telemetry.speak("Drop on D3");
                    break;
            }
            autonomousOptions.setDropLocation(dropLocation);
            teleDropLocation.setValue(dropLocation);
        }

        // Where to make first drop
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
            AutonomousOptions.FirstDrop firstDrop = autonomousOptions.getFirstDrop().getNext();
            switch (firstDrop) {
                case B2:
                    telemetry.speak("First drop on B2");
                    break;
                case C2:
                    telemetry.speak("First drop on C2");
                    break;
                case D2:
                    telemetry.speak("First drop on D2");
                    break;
            }
            autonomousOptions.setFirstDrop(firstDrop);
            teleFirstDrop.setValue(firstDrop);
        }

        // Keep range within 0-15 seconds. Wrap at either end.
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            autonomousOptions.setDelayStartSeconds(autonomousOptions.getDelayStartSeconds() - 1);
            autonomousOptions.setDelayStartSeconds((autonomousOptions.getDelayStartSeconds() < 0) ? 15 : autonomousOptions.getDelayStartSeconds());
            telemetry.speak("delay start " + autonomousOptions.getDelayStartSeconds() + " seconds");
        }
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            autonomousOptions.setDelayStartSeconds(autonomousOptions.getDelayStartSeconds() + 1);
            autonomousOptions.setDelayStartSeconds((autonomousOptions.getDelayStartSeconds() > 15) ? 0 : autonomousOptions.getDelayStartSeconds());
            telemetry.speak("delay start " + autonomousOptions.getDelayStartSeconds() + " seconds");
        }
        teleDelayStartSeconds.setValue(autonomousOptions.getDelayStartSeconds());

        //Have the required options been set?
        readyToStart = !(autonomousOptions.getStartPosition() == AutonomousOptions.StartPosition.None);
        teleReadyToStart.setValue(readyToStart);

        //Save the options to a file if ready to start and start button is pressed.
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.START) && getReadyToStart()) {
            SaveOptions();
            savedToFile = true;
            teleSavedToFile.setValue(true);
            doneFlag = true;
        }

        return doneFlag;
    }

    // Default selections if driver does not select anything.
    private void resetOptions() {
        autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.None);
        autonomousOptions.setDropLocation(AutonomousOptions.DropLocation.D3);
        autonomousOptions.setFirstDrop(AutonomousOptions.FirstDrop.C2);
        autonomousOptions.setDelayStartSeconds(0);
        readyToStart = false;
        savedToFile = false;
    }

    private void SaveOptions() {
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        readWriteAutoOptions.storeObject(autonomousOptions);
    }

    public AutonomousOptions getSaveAutoOptions() {
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        AutonomousOptions temp = readWriteAutoOptions.getObject();
        telemetry.addData("Start: ", temp.getStartPosition());
        telemetry.update();
        return temp;
    }
}