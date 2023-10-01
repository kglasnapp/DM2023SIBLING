
package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.OperatorButtons;
import frc.robot.RobotContainer.RobotMode;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DisplayLogCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.State;
import frc.robot.commands.PositionCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.commands.RobotOrientedDriveDeacceleratedCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.SetModeConeCube;
import frc.robot.commands.StraightPathCommand;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.KeyPadPositionSupplier;

public class Autonomous {
  DrivetrainSubsystem drivetrainSubsystem;
  RobotContainer robotContainer;
  private final BalanceCommand balanceCommand;

  public static Command test;

  public Autonomous(RobotContainer robotContainer, DrivetrainSubsystem drivetrain, IntakeSubsystem intake) {
    this.robotContainer = robotContainer;
    this.drivetrainSubsystem = drivetrain;
    balanceCommand = new BalanceCommand(drivetrain);
    RobotContainer.autonomousChooser.setDefaultOption("Case 1 outside", getAutonomousCommandCase1());
    RobotContainer.autonomousChooser.addOption("Case 2 left turn", getAutonomousCommandCase2());
    RobotContainer.autonomousChooser.addOption("Case 2 right turn", getAutonomousCommandCase2());
    RobotContainer.autonomousChooser.addOption("Case 3 Center Balance", getAutonomousCommandCase3());
    RobotContainer.autonomousChooser.addOption("Case 4 Center Pickup Balance", getAutonomousCommandCase4());
    RobotContainer.autonomousChooser.addOption("Test Movements", testMovements(drivetrain, intake, robotContainer));
    RobotContainer.autonomousChooser.addOption("Balance ", balanceCommand);

    test = testMovements(drivetrain, intake, robotContainer);
    // Put the chooser on the dashboard
    SmartDashboard.putData("Autonomous Mode", RobotContainer.autonomousChooser);
    //SmartDashboard.putData("Autonomous Mode", autonomousChooser);

  }

  private Command getAutonomousCommandCase1() {
    CommandBase command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))
        .andThen(new WaitCommand(0.5))
        .andThen(new DisplayLogCommand("Run Case 1"));
    return command;
  }

  private Command getAutonomousCommandCase2() {
    CommandBase command = new DisplayLogCommand("Case 2")
        .andThen(new WaitCommand(0.5));
    return command;
  }

  private Command getAutonomousCommandCase3() {
    CommandBase command = new DisplayLogCommand("Case 3")
        .andThen(new WaitCommand(0.5));
    return command;
  }

  private Command getAutonomousCommandCase4() {
    CommandBase command = new DisplayLogCommand("Case 4")
        .andThen(new WaitCommand(0.5));
    return command;
  }

  private Command testMovements(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intake,
      RobotContainer robotContainer) {
    return new DisplayLogCommand("Test Case")
        //.andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, .4, 0, 0, 500))
        .andThen(new SetModeConeCube(RobotMode.Cube))
        //.andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, 0, 0, 0, 50))
        //.andThen(new PositionCommand(robotContainer, OperatorButtons.LOW))
        .andThen(new IntakeCommand(intake, State.OUT, 1000))
        .andThen(new WaitCommand(.5))
        .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))
        .andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, -.3, 0, 0, 1000))
        .andThen(new WaitCommand(.25))
        .andThen(new RotateCommand(drivetrainSubsystem))
        .andThen(new PositionCommand(robotContainer, OperatorButtons.GROUND))
        .andThen(new IntakeCommand(intake, State.IN, 1000))
        .andThen(new RobotOrientedDriveDeacceleratedCommand(drivetrainSubsystem, -.3, 0, 0, 300))
        ;
    // .andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, 0, 0, 0, 50));
  }

  private Supplier<Pose2d> poseEstimator; // TODO Elie how do we get a pose provider

  public Command test2() {
    CommandBase command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))
        .andThen(new StraightPathCommand(drivetrainSubsystem, poseEstimator,
            new Pose2d(new Translation2d(2.1, KeyPadPositionSupplier.FIELD_WIDTH - 5.24),
                new Rotation2d(Math.toRadians(180)))))
        .andThen(new DriveCommand(drivetrainSubsystem, -2, 0, 100))
        .andThen(new WaitCommand(0.5))
        .andThen(new DriveCommand(drivetrainSubsystem, -2, 0, 0))
        .andThen(new WaitCommand(2.5))
        .andThen(new DriveCommand(drivetrainSubsystem, -0.05, 0, 0))
        .andThen(new WaitCommand(2))
        .andThen(new DriveCommand(drivetrainSubsystem, 0, 0, 0))
        .andThen(new BalanceCommand(drivetrainSubsystem));
    command.setName("Test 2");
    return command;
  }
  // autonomousChooser.setDefaultOption("Over and Balance",

  // AutonomousCommandFactory.getAutonomousSimpleLowCommand(m_drivetrainSubsystem,
  // m_armSubsystem, grabberSubsystem)
  // .andThen(AutonomousCommandFactory.getOverAndBalanceCommand(m_drivetrainSubsystem,
  // poseEstimator)));
  // // A chooser for autonomous commands
  // autonomousChooser.setDefaultOption("Middle Balance",
  // AutonomousCommandFactory.getAutonomousSimpleCommand(m_drivetrainSubsystem,
  // m_armSubsystem, grabberSubsystem)
  // .andThen(AutonomousCommandFactory.getSetPositionAndBalanceCommand(m_drivetrainSubsystem,
  // poseEstimator)));
  // autonomousChooser.addOption("Simple Case and Left out",
  // AutonomousCommandFactory.getAutonomousAcceleratedAndLeftOutCommand(m_drivetrainSubsystem,
  // m_armSubsystem,
  // grabberSubsystem));
  // autonomousChooser.addOption("Simple Case and Right out",
  // AutonomousCommandFactory.getAutonomousSimpleAndRightDeacceleratedOutCommand(m_drivetrainSubsystem,
  // m_armSubsystem,
  // grabberSubsystem));

  // // Add commands to the autonomous command chooser
  // autonomousChooser.addOption("Case 1 left",
  // getAutonomousCommandCase1(0).andThen(new
  // StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 2),
  // getFinalPoseForCase1(0))));

  // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
  // autonomousChooser.addOption("CaSe 1 right", getAutonomousCommandCase1(2)
  // .andThen(new StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 0),
  // getFinalPoseForCase1(2))));

  // autonomousChooser.addOption("Case 2 left", getAutonomousCommandCase2(0));
  // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
  // autonomousChooser.addOption("Case 2 red", getAutonomousCommandCase2Red());
  // autonomousChooser.addOption("Case 2 blue", getAutonomousCommandCase2Blue());
  // autonomousChooser.addOption("Case 2 right", getAutonomousCommandCase2(2));
  // autonomousChooser.addOption("Case 3", getAutonomousCommandCase3());

  public Command getAutonomousCommandCase2Red() {
    CommandBase command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))

        // .andThen(new GrabberCommand(grabberSubsystem, false))
        // .andThen(new KeyPadStateCommand(1))
        // .andThen(getCommandFor(1))
        // .andThen(new GrabberCommand(grabberSubsystem, true))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new ZeroExtenderCommand(m_armSubsystem))
        // .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        // new Pose2d(new Translation2d(2.1, 5.24), new
        // Rotation2d(Math.toRadians(180)))))
        // .andThen(new ZeroShoulderCommand(m_armSubsystem))
        // .andThen(new ChangeTurboModeCommand())
        // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        .andThen(new WaitCommand(0.5));
    // .andThen(new ChangeNormalModeCommand())
    // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
    // .andThen(new WaitCommand(2.5))
    // // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
    // // .andThen(new WaitCommand(2))
    // .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
    // // .andThen(new BalanceCommand(m_drivetrainSubsystem))
    command.setName("Case 2 red");
    return command;
  }

  // public Command getAutonomousCommandCase2Blue() {
  // CommandBase command = new ZeroGyroCommand(m_drivetrainSubsystem,
  // balanceCommand, (180))
  // .andThen(new GrabberCommand(grabberSubsystem, false))
  // .andThen(new KeyPadStateCommand(1))
  // .andThen(getCommandFor(1))
  // .andThen(new GrabberCommand(grabberSubsystem, true))
  // .andThen(new WaitCommand(0.5))
  // .andThen(new ZeroExtenderCommand(m_armSubsystem))
  // .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
  // new Pose2d(new Translation2d(2.1, KeyPadPositionSupplier.FIELD_WIDTH - 5.24),
  // new Rotation2d(Math.toRadians(180)))))
  // .andThen(new ZeroShoulderCommand(m_armSubsystem))
  // .andThen(new ChangeTurboModeCommand())
  // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
  // .andThen(new WaitCommand(0.5))
  // .andThen(new ChangeNormalModeCommand())
  // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
  // .andThen(new WaitCommand(2.5))
  // // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
  // // .andThen(new WaitCommand(2))
  // .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
  // // .andThen(new BalanceCommand(m_drivetrainSubsystem))
  // command.setName("Case 2 blue");
  // return command;
  // }

  // public Command getAutonomousCommandCase3() {
  // KeyPadPositionSupplier.state = 0;
  // Command command = new CommandBase() {
  // @Override
  // public void initialize() {
  // KeyPadPositionSupplier.state = 0;
  // }

  // @Override
  // public boolean isFinished() {
  // return true;
  // }

  // }.andThen(getCommandFor(0)
  // .andThen(new GrabberCommand(grabberSubsystem, true))
  // // .andThen(new WaitCommand(1))
  // .andThen(new ZeroExtenderCommand(m_armSubsystem))
  // // .andThen(
  // // new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
  // // new Pose2d(5.75,
  // // 7.4,
  // // new Rotation2d(Math.toRadians(180)))))
  // .andThen(
  // new StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 0),
  // new Pose2d(4.4,
  // 5.3,
  // new Rotation2d(Math.toRadians(0)))))
  // .andThen(
  // new StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 0),
  // new Pose2d(6.4,
  // 5.3,
  // new Rotation2d(Math.toRadians(0)))))
  // .andThen(new ShoulderCommand(m_armSubsystem, 40352))
  // .andThen(new ExtenderCommand(m_armSubsystem, 183023 * 16 / 36))
  // .andThen(new GrabberCommand(grabberSubsystem, false))
  // .andThen(new GrabberCommand(grabberSubsystem, false))
  // .andThen(new WaitCommand(2))
  // .andThen(new ZeroExtenderCommand(m_armSubsystem))
  // .andThen(Commands.parallel(
  // new ShoulderCommand(m_armSubsystem, 190432),
  // new StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 0),
  // new Pose2d(3.2, 5.3,
  // new Rotation2d(Math.toRadians(0))))))
  // .andThen(getCommandFor(4))
  // .andThen(new GrabberCommand(grabberSubsystem, true)));
  // command.setName("case 3");
  // return command;
  // }

}