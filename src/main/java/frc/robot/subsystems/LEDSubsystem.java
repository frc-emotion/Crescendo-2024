package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * LED Subsystem
 */
public class LEDSubsystem extends SubsystemBase {

    private static final int PWM_PORT = 0;
    private static final int NUM_LEDS = 72;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public static enum LEDStyle {
        BLINK_RAPID,
        BLINK_SLOW,
        BLINK,
        SOLID;

        public static int getDelayAmount(LEDStyle style) {
            switch (style) {
                case BLINK_RAPID:
                    return LEDConstants.FLASH_DELAY / 2;
                case BLINK_SLOW:
                    return LEDConstants.FLASH_DELAY * 2;
                case BLINK:
                    return LEDConstants.FLASH_DELAY;
                default:
                    return 0;
            }
        }
    }

    /**
     * Construct a new LEDSubsystem instance
     */
    public LEDSubsystem() {
        m_led = new AddressableLED(PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
        m_led.setLength(m_ledBuffer.getLength());

        for (int i = 0; i < NUM_LEDS; i++) {
            m_ledBuffer.setRGB(i, 255, 255, 0); // yellow
        }

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /** 
     * Set color of LEDs using Color object
     * @param color Color object reference for setting LED Color
     */
    public void setColor(Color color) {
        for (int i = 0; i < NUM_LEDS; i++) {
            m_ledBuffer.setLED(i, color);
        }
        m_led.setData(m_ledBuffer);
    }

    /** 
     * Set color of LEDs using HSV Value
     * @param h Hue value for setting LED Color (0 to 255)
     * @param s Saturation value for setting LED Color (0 to 255)
     * @param v V-value for setting LED Color (0 to 255)
    */
    public void setHSV(int h, int s, int v) {
        for (int i = 0; i < NUM_LEDS; i++) {
            m_ledBuffer.setHSV(i, h, s, v);
        }
    }

    /**
     * Set color of LEDs using RGB Value
     * @param r Red value for setting LED Color (0 to 255)
     * @param g Green value for setting LED Color (0 to 255)
     * @param b Blue value for setting LED Color (0 to 255)
     */
    public void setRGB(int r, int g, int b) {
        for (int i = 0; i < NUM_LEDS; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }

        m_led.setData(m_ledBuffer);
    }

    /**
     * Turns off all LEDs
     */
    public void turnOff() {
        setRGB(0, 0, 0);
    }

    /**
     * Checks if LEDs are on (only checks first LED)
     * @return true if the first LED is on
     */
    public boolean isOn() {
        Color c = m_ledBuffer.getLED(0);
        return !c.equals(Color.fromHSV(0, 0, 0));
    }
}
