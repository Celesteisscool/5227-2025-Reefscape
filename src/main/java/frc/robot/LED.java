package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;

public class LED {
    public AddressableLED leds;
    public AddressableLEDBuffer data = new AddressableLEDBuffer(158);
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    private static final Distance LedSpacing = Meters.of(1 / 120.0);
    private final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), LedSpacing);

    private AddressableLEDBufferView underBumperSection = data.createView(0, 112);
    private AddressableLEDBufferView elevatorSection = data.createView(112, 157);

    public LED() {
        leds = new AddressableLED(0);
        leds.setLength(data.getLength());
        leds.start();
    }
    
    public void setLEDSAlliance(AddressableLEDBufferView buffer) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        boolean isRedAlliance = false;
        if (DriverStation.getAlliance().isPresent()) {
            isRedAlliance = (ally.get() == DriverStation.Alliance.Red);
        }
        if (isRedAlliance) { LEDPattern.solid(Color.kRed).applyTo(buffer); } 
        else { LEDPattern.solid(Color.kBlue).applyTo(buffer); } 
    }

    public void setLEDSRainbow(AddressableLEDBufferView buffer) {
        scrollingRainbow.applyTo(buffer);
    }

    public void setLEDWhite(AddressableLEDBufferView buffer) {
        for (int i = 0; i < buffer.getLength(); i = i+1) {
            buffer.setRGB(i, 255, 255, 255);
        }
    }

    public void setRotationLED(AddressableLEDBufferView buffer, double angle) {
        double conversion = (28*4);
        
        if (angle < 0) {
            angle += 360;
        }

        int index = ((int)Math.round((angle/360) * conversion));
        
        setLEDSAlliance(buffer);
        buffer.setRGB(index, 255, 255, 255);

    }


    public void setAllLED(int mode) {
        boolean ALIGNING = Teleop.ALIGNING;
        if (mode == 1) { // TELEOP
            setRotationLED(underBumperSection, Teleop.joystickAngle);
            if (ALIGNING) {
                setLEDWhite(elevatorSection);
            }
            else {
                setLEDSRainbow(elevatorSection);
            } 
        } 
        else if (mode == 0) { // DISABLED
            setLEDSAlliance(underBumperSection);
            setLEDSRainbow(elevatorSection);
        }
        leds.setData(data);        
    } 


    
}
