// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
/*import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;*/
import edu.wpi.first.wpilibj.XboxController;
/*import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;*/
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoralShootCommand;
//import frc.robot.commands.TrajectoryCommand;
import frc.robot.commands.CoralShootCommand.CoralLevel;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystemL2;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import java.util.List;
//import java.util.function.BooleanSupplier;
//import java.util.function.Consumer;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

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
    public final static CoralSubsystemL2 coralSubsystemL2 = new CoralSubsystemL2();

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

        NamedCommands.registerCommand("LEVEL1", new CoralShootCommand(CoralLevel.LEVEL1));
        
        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            //field.setRobotPose(pose);
            //System.out.println("current x,y:" + pose.getX() + "," + pose.getY());
        });

        // Configure default commands
        drive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> drive.drive(
                    // Removed minus sign to flip robot to L1 coral = front. NG - switched the joystick directions to make field-oriented drive oriented
                    MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
                    MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
                    MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband),
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
        
        m_driverController.povUp().whileTrue(Commands.run(() -> drive.setX(), drive));
        m_driverController.y().onTrue(Commands.runOnce(() -> {
            isFieldRelative = !isFieldRelative; // Toggle the mode
            System.out.println("Drive Mode: " + (isFieldRelative ? "Field Relative" : "Robot Relative"));
        }, drive));
        m_driverController.rightBumper().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL1));
        m_driverController.rightTrigger().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL2SHOOT));
        m_driverController.a().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL1SLOW));
        m_driverController.leftBumper().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL2F));
        m_driverController.leftTrigger().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL2B));
        m_driverController.x().whileTrue(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL1FULLFIRE));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(String choice) {
        if (choice.startsWith("Simple"))
            return null;
        
        return new PathPlannerAuto(choice);
    }
}


/*Old version with trajectories
public Command getAutonomousCommand(String choice) {
    if (choice.startsWith("Simple"))
        return null;

    Configs.TrajectoryConfigs.thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //Center trajectory
    Trajectory centerL1Trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X direction
        List.of(new Translation2d(-0.6, 0)), //we need at least one waypoint?
        new Pose2d(-1.22, 0, new Rotation2d(0)), //this get
        Configs.TrajectoryConfigs.config);

    //Left trajectory (starting left of center from robot's perspective)
    Trajectory leftL1Trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X direction
        List.of(new Translation2d(-0.6, 0)), //we need at least one waypoint?
        new Pose2d(-2.22, 1.4, new Rotation2d(1.047)), //this is a 60 degree right turn
        Configs.TrajectoryConfigs.config);

    //Right trajectory (starting right of center from robot's perspective)
    Trajectory rightL1Trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X direction
        List.of(new Translation2d(-0.6, 0)), //we need at least one waypoint?
        new Pose2d(-2.22, -1.4, new Rotation2d(-1.047)), //this is a 60 degree left turn
        Configs.TrajectoryConfigs.config);

    Trajectory chosenTrajectory = centerL1Trajectory; //this is the default
    if (choice == "Left")
        chosenTrajectory = leftL1Trajectory;
    if (choice == "Right")
        chosenTrajectory = rightL1Trajectory;

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        chosenTrajectory,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        Configs.TrajectoryConfigs.XController, 
        Configs.TrajectoryConfigs.YController, 
        Configs.TrajectoryConfigs.thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(chosenTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return null;
    return swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0, false))
        .andThen(new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL1));
}*/
