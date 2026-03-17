package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveForward extends SequentialCommandGroup {

    public DriveForward(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive)
    {
        final var idle = new SwerveRequest.Idle();

        addRequirements(drivetrain);

        addCommands(// Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle));
    }
}
