// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.OperatorConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.auto.DriveForward;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed); 

    // The robot's subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final FuelSubsystem fuelSubsystem = new FuelSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // The driver's controller
    private final CommandXboxController driverController = new CommandXboxController(
        DRIVER_CONTROLLER_PORT);

    // The operator's controller, by default it is setup to use a single controller
    private final CommandXboxController operatorController = new CommandXboxController(
        OPERATOR_CONTROLLER_PORT);

    // The autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        // Set the options to show up in the Dashboard for selecting auto modes. If you
        // add additional auto modes you can add additional lines here with
        // autoChooser.addOption
        autoChooser.setDefaultOption("Autonomous", new DriveForward(drivetrain, drive));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // While the left bumper on operator controller is held, intake Fuel
        operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
        //operatorController.leftBumper().whileTrue(new LaunchSequence(fuelSubsystem));

        // While the right bumper on the operator controller is held, spin up for 1
        // second, then launch fuel. When the button is released, stop.
        operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
        //operatorController.rightBumper().whileTrue(new Intake(fuelSubsystem));

        // While the A button is held on the operator controller, eject fuel back out
        // the intake
        operatorController.a().whileTrue(new Eject(fuelSubsystem));

        // While the down arrow on the directional pad is held it will unclimb the robot
        operatorController.povDown().whileTrue(new ClimbDown(climberSubsystem));
        
        // While the up arrow on the directional pad is held it will cimb the robot
        operatorController.povUp().whileTrue(new ClimbUp(climberSubsystem));

        fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

        climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));
    }

    public Command getAutonomousCommand() {
        
        return autoChooser.getSelected();
    }


}
