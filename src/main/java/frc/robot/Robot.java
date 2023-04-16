// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANPIDController;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.ControlType;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PDHData;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static long count = 0;
  private final PDHData pdhData = new PDHData();
  public static Alliance alliance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int color = 0;
  Command cmd;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    alliance = DriverStation.getAlliance();
    Util.logf("Start Swerve %s\n", alliance.toString());

    m_robotContainer = new RobotContainer();
    // testingMotors();
    initNeoPixel();

    CameraServer.startAutomaticCapture();
  }

  SparkMaxPIDController controller;

  // public void testingMotors() {
  // int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
  // CANSparkMax motor = new CANSparkMax(FRONT_RIGHT_MODULE_STEER_MOTOR,
  // CANSparkMaxLowLevel.MotorType.kBrushless);
  // checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0,
  // 100), "Failed set periodic frame period k0");
  // checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
  // 20), "Failed set periodic frame period k1");
  // checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
  // 20), "Failed set periodic frame period k2");
  // checkNeoError(motor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed set
  // iddle mode");
  // // took the value for setInverted from the MK4_L2 ModuleConfiguration
  // motor.setInverted(true);
  // double nominalVoltage = 12.0;
  // double driveCurrentLimit = 80.0;
  // double steerCurrentLimit = 20.0;
  // checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed
  // setting voltage compensation");
  // checkNeoError(motor.setSmartCurrentLimit((int)
  // Math.round(steerCurrentLimit)), "Failed set smart current limit");

  // CANEncoder integratedEncoder = motor.getEncoder();
  // double steerReduction = (15.0 / 32.0) * (10.0 / 60.0);
  // checkNeoError(integratedEncoder.setPositionConversionFactor(2.0 * Math.PI *
  // steerReduction), "Failed to set NEO encoder conversion factor");
  // checkNeoError(integratedEncoder.setVelocityConversionFactor(2.0 * Math.PI *
  // steerReduction / 60.0), "Failed to set NEO encoder conversion factor");
  // // integratedEncoder.setPosition(absoluteEncoder.getAbsoluteAngle()); //,
  // "Failed to set NEO encoder position");

  // controller = motor.getPIDController();

  // checkNeoError(controller.setP(1), "Failed to set NEO PID proportional
  // constant");
  // checkNeoError(controller.setI(0.0), "Failed to set NEO PID integral
  // constant");
  // checkNeoError(controller.setD(0.1), "Failed to set NEO PID derivative
  // constant");

  // // we ask the PID to change the angle to PI/2 = 90 degrees
  // System.out.println("Rotating front right 90 degrees");
  // checkNeoError(controller.setReference(Math.PI/2,
  // CANSparkMax.ControlType.kPosition), "Failed set reference to 90 degrees");
  // System.out.println("Done front right 90 degrees");
  // try {
  // Thread.sleep(5000);
  // } catch (Exception e) {

  // }
  // System.out.println("Rotating front right 180 degrees");
  // checkNeoError(controller.setReference(Math.PI,
  // CANSparkMax.ControlType.kPosition), "Failed set reference to 180 degrees");
  // System.out.println("Done Rotating front right 180 degrees");
  // // }
  // // checkNeoError(controller.setFeedbackDevice(integratedEncoder), "Failed to
  // set NEO PID feedback device");

  // // public static final ModuleConfiguration MK4_L2 = new ModuleConfiguration(
  // // 0.10033,
  // // (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
  // // true,
  // // (15.0 / 32.0) * (10.0 / 60.0),
  // // true
  // // );
  // }

  private void initNeoPixel() {
    m_led = new AddressableLED(9);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  private void setNeoPixelColors() {
    if (count % 50 == 0) {
      color++;
      color = color % 6;
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB
        if (color == 0)
          m_ledBuffer.setRGB(i, 255, 0, 0);
        if (color == 1)
          m_ledBuffer.setRGB(i, 0, 255, 0);
        if (color == 2)
          m_ledBuffer.setRGB(i, 0, 0, 255);
        if (color == 3)
          m_ledBuffer.setRGB(i, 255, 255, 0);
        if (color == 4)
          m_ledBuffer.setRGB(i, 128, 128, 128);
        if (color == 5)
          m_ledBuffer.setRGB(i, 0, 0, 0);
      }
      m_led.setData(m_ledBuffer);
    }
  }

  public static void checkNeoError(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      throw new RuntimeException(String.format("%s: %s", message, error.toString()));
    }
  }

  int i = 0;

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    count++;
    if (count % 500 == 0) {
      pdhData.logPDHData();
    }
    setNeoPixelColors();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (cmd != null) {
      logf("Executing disabled init %s\n", cmd.getName());
      cmd.cancel();
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    cmd = RobotContainer.autonomousChooser.getSelected();
    if (cmd != null) {
      logf("Executing autonomous %s\n", cmd.getName());
      cmd.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Util.logf("TELEOP INIT %s\n", alliance.toString());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
