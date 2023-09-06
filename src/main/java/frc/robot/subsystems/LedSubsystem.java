package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

//import static frc.robot.utilities.Util.logf;

public class LedSubsystem extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    //private int color = 0;

    private ElevatorSubsystem elevatorSubsystem;
    private GrabberTiltSubsystem grabberSubsystem;

    public LedSubsystem(ElevatorSubsystem elevatorSubsystem, GrabberTiltSubsystem grabberSubsystem) {
        
        initNeoPixel();
        this.elevatorSubsystem = elevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;
    }

    public enum Leds  { GrabberForward(8)  , GrabberReverse(9), ElevatorForward(11), ElevatorReverse(10),
        IntakeOverCurrent(20);
        public final int val;
        private Leds(int val) {
            this.val = val;
        }
    };

    @Override
    public void periodic() {
        if (Robot.count % 5 == 0) {
            setNeoPixelColors();
        }
    }

    private void initNeoPixel() {
        m_led = new AddressableLED(9);
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(32);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    // public void setoldNeoPixelColors() {
    //     if (Robot.count % 50 == 0) {
    //         color++;
    //         color = color % 6;
    //         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //             // Sets the specified LED to the RGB
    //             if (color == 0)
    //                 m_ledBuffer.setRGB(i, 80, 0, 0);
    //             if (color == 1)
    //                 m_ledBuffer.setRGB(i, 0, 80, 0);
    //             if (color == 2)
    //                 m_ledBuffer.setRGB(i, 0, 0, 80);
    //             if (color == 3)
    //                 m_ledBuffer.setRGB(i, 80, 40, 0);
    //             if (color == 4)
    //                 m_ledBuffer.setRGB(i, 128, 128, 128);
    //             if (color == 5)
    //                 m_ledBuffer.setRGB(i, 0, 0, 0);
    //         }
    //         m_led.setData(m_ledBuffer);
    //     }

    // }

    public void setLimitSwitchLed(Leds led, boolean value) {
        if (value) {
            m_ledBuffer.setRGB(led.val, 80, 0, 0);
        } else {
            m_ledBuffer.setRGB(led.val, 0, 80, 0);
        }
    }



    private void setNeoPixelColors() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            if (i < 5) {
                // Aliance color leds
                if (Robot.alliance == Alliance.Red)
                    m_ledBuffer.setRGB(i, 80, 0, 0);
                else if (Robot.alliance == Alliance.Blue)
                    m_ledBuffer.setRGB(i, 0, 0, 80);
                else
                    m_ledBuffer.setRGB(i, 80, 80, 80);
            } else if (i < 6) {
                // White sepatrator
                m_ledBuffer.setRGB(i, 80, 80, 80);
            } else if (i < 8) {
                // Elevator forward limit
                if (elevatorSubsystem.getForwardLimitSwitch()) {
                    m_ledBuffer.setRGB(i, 80, 0, 0);
                } else {
                    m_ledBuffer.setRGB(i, 0, 80, 0);
                }
            } else if (i < 9) {
                // White sepatrator
                m_ledBuffer.setRGB(i, 80, 80, 80);
            } else if (i < 11) {
                // Elevator reverse limit
                if (elevatorSubsystem.getReverseLimitSwitch()) {
                    m_ledBuffer.setRGB(i, 80, 0, 0);
                } else {
                    m_ledBuffer.setRGB(i, 0, 80, 0);
                }
            } else if (i < 13) {
                // White sepatrator
                m_ledBuffer.setRGB(i, 80, 80, 80);
            } else if (i < 15) {
                // Grabber tilt forward limit
                if (grabberSubsystem.getForwardLimitSwitchTilt()) {
                    m_ledBuffer.setRGB(i, 80, 0, 0);
                } else {
                    m_ledBuffer.setRGB(i, 0, 80, 0);
                }
            } else if (i < 16) {
                // White sepatrator
                m_ledBuffer.setRGB(i, 80, 80, 80);
            } else if (i < 18) {
                // Grabber tilt reverse limit
                if (grabberSubsystem.getReverseLimitSwitchTilt()) {
                    m_ledBuffer.setRGB(i, 80, 0, 0);
                } else {
                    m_ledBuffer.setRGB(i, 0, 80, 0);
                }
            } else {
                // Robot mode
                if (RobotContainer.robotMode == RobotContainer.RobotMode.Cube)
                    m_ledBuffer.setRGB(i, 80, 0, 80);
                else 
                    m_ledBuffer.setRGB(i, 80, 80, 0);
            }
        }
        m_led.setData(m_ledBuffer);
    }
}
