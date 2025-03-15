package frc.robot;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class LED {
    public AddressableLED leds;
    public AddressableLEDBuffer data = new AddressableLEDBuffer(24*2);

    private final Distance LED_SPACING = Meters.of(1.0 / 60);

    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    private final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(InchesPerSecond.of(12), LED_SPACING);

    private final LEDPattern purpleGold = LEDPattern.gradient(GradientType.kContinuous, new Color[] {
        new Color(75,0,130), // Purple
        new Color(255, 223, 0) } ); // Gold
    private final LEDPattern purpleGoldScrolling = purpleGold.scrollAtAbsoluteSpeed(InchesPerSecond.of(12), LED_SPACING);


    private final LEDPattern patternRed = LEDPattern.solid(new Color(100,0,0));
    private final LEDPattern patternGreen = LEDPattern.solid(new Color(0,100,0));
    private final LEDPattern patternBlue = LEDPattern.solid(new Color(0,0,100));
    private final LEDPattern patternOff = LEDPattern.kOff;
    private final LEDPattern patternWhite = LEDPattern.solid(new Color(100,100,100));
    private final LEDPattern patternPurple = LEDPattern.solid(Color.kViolet);


    private final AddressableLEDBufferView rightBack = new AddressableLEDBufferView(data, 0, 5);
    private final AddressableLEDBufferView leftBack = new AddressableLEDBufferView(data, 18, 23);
    
    private final AddressableLEDBufferView rightFront = new AddressableLEDBufferView(data, 24, 35);
    private final AddressableLEDBufferView leftFront = new AddressableLEDBufferView(data, 35, 47);



    public double AlignSide = 0; // true is left false is right
    public double leftAlign = -2.25;    
    
    public LED() {
        leds = new AddressableLED(0);
        leds.setLength(data.getLength());
        leds.start();
    }

    public void setLEDOff() {
        patternOff.applyTo(data);
    }

    public void setLEDRED() {
        patternRed.applyTo(data);
    }

    public void setLEDGREEN() {
        patternGreen.applyTo(data);
    }

    public void setLEDBLUE() {
        patternBlue.applyTo(data);
    }

    public void setLEDRainbow() {
        scrollingRainbow.applyTo(data);
    }

    public void setLEDPurpleGold() {
        purpleGoldScrolling.atBrightness(Percent.of(25)).applyTo(data);
    }

    public void setLEDMoveLeft(int strength) {
        Dimensionless strengthPer = Percent.of(strength);
        
        patternWhite.atBrightness(strengthPer).applyTo(rightFront);
        patternWhite.atBrightness(strengthPer).applyTo(rightBack);
        // patternWhite.atBrightness(Percent.of(0)).applyTo(leftFront);
        // patternWhite.atBrightness(Percent.of(0)).applyTo(leftBack);
    }

    public void setLEDMoveRight(int strength) {
        Dimensionless strengthPer = Percent.of(strength);
        patternWhite.atBrightness(strengthPer).applyTo(leftFront);
        patternWhite.atBrightness(strengthPer).applyTo(leftBack);
        // patternWhite.atBrightness(Percent.of(0)).applyTo(rightFront);
        // patternWhite.atBrightness(Percent.of(0)).applyTo(rightBack);
    }

    public void setLEDPickup() {
        patternGreen.applyTo(data);
    }

    public void setLEDMoveOff() {
        patternWhite.atBrightness(Percent.of(0)).applyTo(leftBack);
        patternWhite.atBrightness(Percent.of(0)).applyTo(leftFront);
        patternWhite.atBrightness(Percent.of(0)).applyTo(rightBack);
        patternWhite.atBrightness(Percent.of(0)).applyTo(rightFront);
    } 

    public void setOne(int id) {
        data.setLED(id, new Color().kCoral);
    }

    public void update() {
        leds.setData(data);
    }    
}
