package org.firstinspires.ftc.teamcode;


import java.io.Serializable;

/*
This class stores autonomous options for use by autonomous or teleop op modes.
It is currently set for the PowerPlay season but can be modified for upcoming seasons.
It is a Java Serializable class to it can be saved to a file.
 */
public class AutonomousOptions implements Serializable {
    private static final long serialVersionUID = 7829136421241571165L;

    private int delayStartSeconds;
    private StartPosition startPosition;
    private DropLocation droplocation;
    private FirstDrop firstDrop;


    public StartPosition getStartPosition() {
        return startPosition;
    }

    public void setStartPosition(StartPosition startPosition) {
        this.startPosition = startPosition;
    }

    public DropLocation getDropLocation() {
        return droplocation;
    }

    public void setDropLocation(DropLocation droplocation) {
        this.droplocation = droplocation;
    }

    public FirstDrop getFirstDrop() {
        return firstDrop;
    }

    public void setFirstDrop(FirstDrop firstDrop) {
        this.firstDrop = firstDrop;
    }

    public int getDelayStartSeconds() {
        return delayStartSeconds;
    }

    public void setDelayStartSeconds(int delayStartSeconds) {
        this.delayStartSeconds = delayStartSeconds;
    }

    /*
     * Where do we start the robot
     * Right is on the right of your substation.
     * Left is on the left of your substation.
     */
    public enum StartPosition {
        None,
        Left,
        Right
    }

    /*
     * Where do we drop. Default is D3.
     */
    public enum DropLocation {
        B2,
        B3,
        D2,
        D3;

        public DropLocation getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Pick which pole for the initial drop
     */
    public enum FirstDrop {
        B2,
        C2,
        D2;

        public FirstDrop getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

}