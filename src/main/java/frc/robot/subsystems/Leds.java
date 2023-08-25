package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import static frc.robot.utilities.Util.logf;

public class Leds extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int color = 0;

    public Leds() {
        initNeoPixel();
    }

    @Override
    public void periodic() {
        setNeoPixelColors();
    }

    private void initNeoPixel() {
        m_led = new AddressableLED(9);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(32);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setoldNeoPixelColors() {
        if (Robot.count % 50 == 0) {
            color++;
            color = color % 6;
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB
                if (color == 0)
                    m_ledBuffer.setRGB(i, 80, 0, 0);
                if (color == 1)
                    m_ledBuffer.setRGB(i, 0, 80, 0);
                if (color == 2)
                    m_ledBuffer.setRGB(i, 0, 0, 80);
                if (color == 3)
                    m_ledBuffer.setRGB(i, 80, 40, 0);
                if (color == 4)
                    m_ledBuffer.setRGB(i, 128, 128, 128);
                if (color == 5)
                    m_ledBuffer.setRGB(i, 0, 0, 0);
            }
            m_led.setData(m_ledBuffer);
        }

    }

    // Code to set half of the Leds to Red and other half to green
    private void setNeoPixelColors() {
        if (Robot.count % 50 == 0) {
            for (var i = 10; i < m_ledBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB
                if (i >= 21)
                    m_ledBuffer.setRGB(i, 80, 0, 0);
                if (i < 21)
                    m_ledBuffer.setRGB(i, 0, 80, 0);
            }
            m_led.setData(m_ledBuffer);
        }
    }

    public void setLed(int led, int r, int g, int b) {
        if (led < 10) {
            m_ledBuffer.setRGB(led, r, g, b);
            m_led.setData(m_ledBuffer);
        } else {
            logf("!!!!!!!!!!!! Invalid LED !!!!!!!!!! %d\n", led);
        }
    }

    public void setLimit(int led, boolean forwardLimit) {
        if(forwardLimit){
            Robot.led.setLed(led, 80, 0, 0);
        } else {
            Robot.led.setLed(led, 0, 80, 0);
        }
    }

    // public static void checkNeoError(REVLibError error, String message) {
    // if (error != REVLibError.kOk) {
    // throw new RuntimeException(String.format("%s: %s", message,
    // error.toString()));
    // }
    // }

}
