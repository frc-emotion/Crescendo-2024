package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LED Subsystem
 *
 * @author Jason Ballinger
 * @version 3/19/2024
 */
public class LEDSubsystem extends SubsystemBase {

    private static final int PWM_PORT = 0;
    private static final int NUM_LEDS = 72;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;



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

    public void setOrange() {
        setRGB(255, 40, 0); // Orange
    }

    public void setGreen() {
        setRGB(0, 255, 0); // Orange
    }

    public void setBlue() {
        setRGB(0, 0, 255); // Blue
    }

    public void setYellow() {
        setRGB(255, 255, 0); // Yellow
    }

    public void setRed() {
        setRGB(255, 0, 0); // Red
    }

    public void setHSV(int h, int s, int v) {
        for (int i = 0; i < NUM_LEDS; i++) {
            m_ledBuffer.setHSV(i, h, s, v);
        }
    }

    public void setRGB(int r, int g, int b) {
        for (int i = 0; i < NUM_LEDS; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }

        m_led.setData(m_ledBuffer);
    }

    public void turnOff() {
        setRGB(0, 0, 0);
    }

    public boolean isOn() {
        Color c = m_ledBuffer.getLED(0);
        return !c.equals(Color.fromHSV(0, 0, 0));
    }
}
