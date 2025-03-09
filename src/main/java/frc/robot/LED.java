package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

public class LED {
    public AddressableLED leds;
    public AddressableLEDBuffer data = new AddressableLEDBuffer(24);
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    private static final Distance LedSpacing = Meters.of(1 / 120.0);
    public double AlignSide = 0; // true is left false is right
    public double leftAlign = -2.25;


    public LED() {
        leds = new AddressableLED(0);
        leds.setLength(data.getLength());
        leds.start();
    }

    public void setLeft(AddressableLEDBuffer buffer, double strength) {
        for (int i = 0; i < buffer.getLength(); i = i+1) {
            int output = (int) Math.floor(strength * 100);
            buffer.setRGB(i,0,0,output);
        }
    }
    public void setRight(AddressableLEDBuffer buffer, double strength) {
        int start = buffer.getLength() - 5;
        for (int i = 0; i < buffer.getLength(); i = i+1) {
            int output = (int) Math.floor(strength * 100);
            buffer.setRGB(i,output,0,0);
        }
    }

    public void setLEDOff(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i = i+1) {
            buffer.setRGB(i,0,0,0);
        }
    }

    public void setLEDMove(double yaw, boolean sideLeft) {
        if (sideLeft) {
            if (yaw > leftAlign) {
                double strength = yaw / 15;
                strength = Math.min(1, strength);
                setLeft(data, strength);
            } else {
                double strength = Math.abs(yaw) / 15;
                strength = Math.min(1, strength);
                setRight(data, strength);
            }
        }
    }



    public void setAllLED(int mode) {

        // if (mode == 1) { // TELEOP
            // double yaw = Constants.visionClass.getClosestYaw();
            // if (yaw != 5227) {
                // setLEDMove(yaw, AlignSide);
            // }
        // } 
        // else if (mode == 0) { // DISABLED
            // setLeft(data, 1);
        // }
        setLEDOff(data);
        leds.setData(data);        
    } 


    
}
