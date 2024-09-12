package org.bitbuckets.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

import java.util.Map;


public class ledSubsystem implements Subsystem, IMattlibHooked {

    // Rev blinkin led driver uses pwm, pretend it is a spark motor
    public final Spark ledController;
    private Map<String, Double> colorValues = Map.of(
            "red", 0.61,
            "orange", 0.65,
            "yellow", 0.69,
            "green", 0.77,
            "blue", 0.87,
            "violet", 0.91,
            "white", 0.93,
            "black", 0.99
    );

    public ledSubsystem(Spark ledController){
        this.ledController = ledController;
    }

    public boolean setColor(String color) {
        if (!colorValues.containsKey(color)) return false;

        ledController.set(colorValues.get(color));
        return true;
    }


}
