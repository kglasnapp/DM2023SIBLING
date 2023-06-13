package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.SwerveModule;

public class DefaultDriveCommand extends CommandBase {

    /**
     * If the robot is running any automation, this property is set to true.
     * In that case, we don't change the Motor Mode (TURBO or NORMAL) when the
     * user presses the controller. We leave it in the state that the autonomous
     * command left it.
     */
    public static boolean autonomous;

    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier precisionActivator;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier precisionActivator) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.precisionActivator = precisionActivator;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        
        if (!autonomous) {
            if (precisionActivator.getAsBoolean()) {
                SwerveModule.powerRatio = SwerveModule.NORMAL;
            } else if (SwerveModule.powerRatio == SwerveModule.NORMAL) {
                SwerveModule.powerRatio = SwerveModule.TURBO;
            }

        }
        if (Robot.count % 20 == 0) {
            if (m_translationXSupplier.getAsDouble() != 0 &&
                    m_translationYSupplier.getAsDouble() != 0) {
                logf("Robot Oriented Speed X: %.2f y:%.2f angle:%.2f\n", m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble());
            }
        }
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
