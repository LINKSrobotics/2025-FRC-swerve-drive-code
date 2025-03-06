// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoralShootCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystemL2;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    // The robot's subsystems
    public final static DriveSubsystem drive = new DriveSubsystem();
    private boolean isFieldRelative = false;
    public final static CoralSubsystem coralSubsystem = new CoralSubsystem();

    // The driver's controller
    //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Call the configured button bindings
        configureButtonBindings();
        System.out.println("Drive Mode: Robot Relative");

        // Configure default commands
        drive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> drive.drive(
                    // Removed minus sign to flip robot to L1 coral = front
                    MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
                    MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
                    MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    isFieldRelative),
                drive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        /*
        new JoystickButton(m_driverController, Button.kR1.value)
            .whileTrue(new RunCommand(
                () -> drive.setX(),
                drive));

        new JoystickButton(m_driverController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(() -> {
                isFieldRelative = !isFieldRelative; // Toggle the mode
                System.out.println("Drive Mode: " + (isFieldRelative ? "Field Relative" : "Robot Relative"));
            }));

        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
            .whileTrue(new RunCommand(() -> coralSubsystem.setMotorSpeed(0.5), coralSubsystem))
            .onFalse(new RunCommand(coralSubsystem::stopMotor, coralSubsystem));

        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new RunCommand(() -> coralSubsystem.setMotorSpeed(-0.5), coralSubsystem))
            .onFalse(new RunCommand(coralSubsystem::stopMotor, coralSubsystem));
        */
        
        m_driverController.povUp().whileTrue(Commands.run(() -> drive.setX(), drive));
        m_driverController.y().onTrue(Commands.runOnce(() -> {
            isFieldRelative = !isFieldRelative; // Toggle the mode
            System.out.println("Drive Mode: " + (isFieldRelative ? "Field Relative" : "Robot Relative"));
        }, drive));
        m_driverController.rightBumper().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL1));
        m_driverController.leftBumper().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL2));
        m_driverController.b().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL2SLOW));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Configs.TrajectoryConfigs.thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-0.3048, 0, new Rotation2d(0)),
            Configs.TrajectoryConfigs.config);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            Configs.TrajectoryConfigs.XController, 
            Configs.TrajectoryConfigs.YController, 
            Configs.TrajectoryConfigs.thetaController,
            drive::setModuleStates,
            drive);

        // Reset odometry to the starting pose of the trajectory.
        drive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0, false));

        /*
        return new TrajectoryCommand(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-0.3048, 0, new Rotation2d(0)));
        */
    }
}
