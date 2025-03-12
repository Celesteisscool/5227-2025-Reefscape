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

    public void setLEDOff() {
        for (int i = 0; i < data.getLength(); i = i+1) {
            data.setRGB(i,0,0,0);
        }
        leds.setData(data);
    }

    public void setLEDRED() {
        for (int i = 0; i < data.getLength(); i = i+1) {
            data.setRGB(i,100,0,0);
        }
        leds.setData(data);  
    }

    public void setLEDGREEN() {
        for (int i = 0; i < data.getLength(); i = i+1) {
            data.setRGB(i,0,100,0);
        }
        leds.setData(data);  
    }

    public void setLEDBLUE() {
        for (int i = 0; i < data.getLength(); i = i+1) {
            data.setRGB(i,0,0,100);
        }
        leds.setData(data);  
    }


    


    
}
