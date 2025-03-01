package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CoralShootCommand extends Command {

    public enum CoralLevel {
        LEVEL1 (0.5),
        LEVEL2 (-0.5),
        LEVEL2SLOW (-.1);

        private double speed;

        private CoralLevel (double s) {
            this.speed = s;
        }

        public double getSpeed() {
            return speed;
        }
    }

    CoralLevel level;

    public CoralShootCommand(CoralLevel level) {
        this.level = level;
        this.addRequirements(RobotContainer.coralSubsystem);
    }

    public void initialize() {}

    public void execute() {
        RobotContainer.coralSubsystem.setMotorSpeed(this.level.getSpeed());
    }

    public void end(boolean interrupted) {
        RobotContainer.coralSubsystem.stopMotor();;
    }
}
