package frc.robot.subsystems;

import static frc.robot.Util.logf;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotMode;
//import static frc.robot.utilities.Util.logf;

public class LedSubsystem extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private boolean change = true;

    //private ElevatorSubsystem elevatorSubsystem;
    // private GrabberTiltSubsystem grabberSubsystem;

    public LedSubsystem() {
        initNeoPixel();
    }

    public enum Leds {
        RobotAlliance(0, 8), GrabberForward(10, 1), GrabberReverse(11, 1), ElevatorForward(13, 1),
        ElevatorReverse(14, 1),  RobotMode(16, 9), IntakeOverCurrent(27, 3);

        public final int val;
        public final int number;

        private Leds(int val, int number) {
            this.val = val;
            this.number = number;
        }
    };

    @Override
    public void periodic() {
        //if (Robot.count % 5 == 0) {
            if (change) {
                m_led.setData(m_ledBuffer);
				change = false;
            }
            
        //}
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

    private void setColors(Leds led, int r, int g, int b) {
        for (int i = led.val; i < led.val + led.number; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        change = true;
    }

    public void setLimitSwitchLed(Leds led, boolean value) {
        if (value) {
            setColors(led, 80, 0, 0);
        } else {
            setColors(led, 0, 80, 0);
        }
    }

    public void setAllianceLeds() {
        if (Robot.alliance == Alliance.Red) {
            setColors(Leds.RobotAlliance, 80, 0, 0);
        } else {
            setColors(Leds.RobotAlliance, 0, 0, 80);
        }
    }

    public void setRobotModeLeds() {
        if (RobotContainer.robotMode == RobotMode.Cube) {
            setColors(Leds.RobotMode, 80, 0, 80);
        } else {
            setColors(Leds.RobotMode, 80, 80, 0);
        }
    }

    public void setOverCurrent(Leds led, boolean value) {
        logf("**** set over current led %b\n", value);
        if (value) {
            setColors(led, 80, 0, 0);
        } else {
            setColors(led, 0, 80, 0);
        }
    }
}
