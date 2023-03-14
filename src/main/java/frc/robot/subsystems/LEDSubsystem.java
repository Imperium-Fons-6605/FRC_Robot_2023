package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    private int m_Hue = 255;
    private int m_Saturation = 255;
    private int m_Value = 255;
    private LEDMode m_mode = LEDMode.CUSTOM;

    public enum LEDMode{
        RAINBOW, CUSTOM, BLINKING
    }

    private int m_rainbowFirstPixelHue;
    private int RGBColor;
    
    public LEDSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    @Override
    public void periodic() {
        super.periodic();
        if (m_mode == LEDMode.RAINBOW){
            fillRainbow();
        } 
        m_led.setData(m_ledBuffer);

    }

    public void fillRainbow(){
        // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
    }

    public void setRainbow(){
        m_mode = LEDMode.RAINBOW;
    }

    public void setCustomColor(int hue, int saturation, int value) {
        m_mode = LEDMode.CUSTOM;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, hue, saturation, value);
          }
    }

}
