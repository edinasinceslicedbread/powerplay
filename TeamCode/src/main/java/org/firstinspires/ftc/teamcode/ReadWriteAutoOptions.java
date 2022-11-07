package org.firstinspires.ftc.teamcode;

import android.content.Context;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

/**
 This class reads or writes AutonomousOptions objects to a file.
 */
public class ReadWriteAutoOptions {
    private final String fileName = "autosettings.txt";
    private Context context;

    public ReadWriteAutoOptions(Context context) {
        this.context = context;
    }

    // Check to see if options file exists.
    public boolean optionsAreSaved() {
        boolean result = true;
        try {
            context.openFileInput(fileName);
        } catch (FileNotFoundException e) {
            result = false;
        }
        return result;
    }

    public void storeObject(AutonomousOptions autonomousOptions) {
        try {
            ObjectOutputStream objectOutputStream = new ObjectOutputStream(context.openFileOutput(fileName, Context.MODE_PRIVATE));
            objectOutputStream.writeObject(autonomousOptions);
            objectOutputStream.flush();
            objectOutputStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public AutonomousOptions getObject() {
        ObjectInputStream objectInputStream;
        AutonomousOptions autonomousOptions = null;
        try {
            FileInputStream fileInputStream = context.openFileInput(fileName);
            objectInputStream = new ObjectInputStream(fileInputStream);
            autonomousOptions = (AutonomousOptions) objectInputStream.readObject();
            objectInputStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return autonomousOptions;
    }
}