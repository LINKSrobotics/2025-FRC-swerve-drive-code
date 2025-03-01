package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class TrajectoryCommand extends SequentialCommandGroup {

    public TrajectoryCommand(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end) {
        Configs.TrajectoryConfigs.thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Trajectory all units in meters.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            start, interiorWaypoints, end, Configs.TrajectoryConfigs.config);

        addCommands(
            Commands.runOnce(() -> RobotContainer.drive.resetOdometry(start), RobotContainer.drive),
            new SwerveControllerCommand(
                trajectory,
                RobotContainer.drive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
                Configs.TrajectoryConfigs.XController, 
                Configs.TrajectoryConfigs.YController, 
                Configs.TrajectoryConfigs.thetaController,
                RobotContainer.drive::setModuleStates,
                RobotContainer.drive
            ),
            Commands.runOnce(() -> RobotContainer.drive.drive(0,0,0,false), RobotContainer.drive)
        );
    }
}
