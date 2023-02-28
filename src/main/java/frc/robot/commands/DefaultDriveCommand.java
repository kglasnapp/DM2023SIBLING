package frc.robot.commands;

//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.swervedrivespecialties.swervelib.SwerveModuleFactory;

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
    private final CommandXboxController controller;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            CommandXboxController controller) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.controller = controller;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        if (!autonomous) {
            if (controller.leftStick().getAsBoolean()) {
                SwerveModuleFactory.powerRatio = SwerveModuleFactory.NORMAL;
            } else if (SwerveModuleFactory.powerRatio == SwerveModuleFactory.NORMAL) {
                SwerveModuleFactory.powerRatio = SwerveModuleFactory.TURBO;
            }
            // if (controller.povDown().getAsBoolean()) {
            //     SwerveModuleFactory.powerRatio = SwerveModuleFactory.PRECISION;
            // }

            if (controller.povUp().getAsBoolean()) {
                SwerveModuleFactory.powerRatio = SwerveModuleFactory.TURBO;
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
