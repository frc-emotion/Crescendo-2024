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
    private static final int NUM_LEDS = 144;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public LEDSubsystem() {
        m_led = new AddressableLED(PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
        m_led.setLength(m_ledBuffer.getLength());

        for (int i = 0; i < NUM_LEDS; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0); // off
        }

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setRGB(int r, int g, int b) {
        for (int i = 0; i < NUM_LEDS; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }

        m_led.setData(m_ledBuffer);
    }

    public boolean isOn() {
        for (int i = 0; i < NUM_LEDS; i++) {
            Color c = m_ledBuffer.getLED(i);
            if (c != Color.fromHSV(0, 0, 0)) return true;
        }
        return false;
    }
}
