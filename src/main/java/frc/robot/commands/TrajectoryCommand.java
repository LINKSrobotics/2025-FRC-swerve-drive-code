package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class TrajectoryCommand extends Command {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    public TrajectoryCommand() {
        this.addRequirements(RobotContainer.m_robotDrive);
    }

    public void initialize() {}

    public void execute() {}

    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }
}
