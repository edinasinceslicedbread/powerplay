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
    private AllianceColor allianceColor;
    private StartPosition startPosition;
    private DropLocation droplocation;
    private FirstDrop firstDrop;
    private PlaceConeInTerminal placeConeInTerminal;
    private PlaceConesOnJunctions placeConesOnJunctions;

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public void setAllianceColor(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

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

    public PlaceConeInTerminal getPlaceConeInTerminal() {
        return placeConeInTerminal;
    }

    public void setPlaceConeInTerminal(PlaceConeInTerminal placeConeInTerminal) {
        this.placeConeInTerminal = placeConeInTerminal;
    }

    public PlaceConesOnJunctions getPlaceConesOnJunctions() {
        return placeConesOnJunctions;
    }

    public void setPlaceConesOnJunctions(PlaceConesOnJunctions placeConesOnJunctions) {
        this.placeConesOnJunctions = placeConesOnJunctions;
    }

    public int getDelayStartSeconds() {
        return delayStartSeconds;
    }

    public void setDelayStartSeconds(int delayStartSeconds) {
        this.delayStartSeconds = delayStartSeconds;
    }

    public String toString() {
        return "AllianceColor: " + getAllianceColor().toString() + "\nStartLocation: " + getStartPosition().toString();
    }

    /*
     * Alliance color. Default to None so driver must select it.
     */
    public enum AllianceColor {
        None,
        Red,
        Blue
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

    /*
     * Yes means place cones on the junctions.
     * Default is No.
     */
    public enum PlaceConesOnJunctions {
        No,
        Yes;

        public PlaceConesOnJunctions getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Yes means place cone in the terminal.
     *  No is the default.
     */
    public enum PlaceConeInTerminal {
        No,
        Yes;

        public PlaceConeInTerminal getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }
}