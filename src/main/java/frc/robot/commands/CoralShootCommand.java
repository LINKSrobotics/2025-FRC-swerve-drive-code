package frc.robot.commands;

//import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;

public class CoralShootCommand extends Command {

    public enum CoralLevel {
        LEVEL1 (0.5),
        LEVEL1SLOW (.25),
        LEVEL2F (0.95),
        LEVEL2B (-0.95),
        LEVEL1FULLFIRE (1),
        LEVEL2SHOOT (0.45),
        LEVEL1B (-0.25);

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
        if (level == CoralLevel.LEVEL2F || level == CoralLevel.LEVEL2B) {
            this.addRequirements(RobotContainer.coralSubsystemL2);
        }
        else {
            this.addRequirements(RobotContainer.coralSubsystem);
        }
    }

    public void initialize() {}

    public void execute() {
        if (level == CoralLevel.LEVEL2SHOOT) {
            RobotContainer.coralSubsystemL2.setMotorSpeed(this.level.getSpeed() * -1);
            RobotContainer.coralSubsystem.setMotorSpeed(this.level.getSpeed() * -1);
        }
        else if (level == CoralLevel.LEVEL2F || level == CoralLevel.LEVEL2B) {
            RobotContainer.coralSubsystemL2.setMotorSpeed(this.level.getSpeed());
        } else {
            RobotContainer.coralSubsystem.setMotorSpeed(this.level.getSpeed());
        }
    }

    public void end(boolean interrupted) {
        if (level == CoralLevel.LEVEL2SHOOT) {
            RobotContainer.coralSubsystemL2.stopMotor();
            RobotContainer.coralSubsystem.stopMotor();
        } 
        else if (level == CoralLevel.LEVEL2F || level == CoralLevel.LEVEL2B) {
            RobotContainer.coralSubsystemL2.stopMotor();
        } else {
            RobotContainer.coralSubsystem.stopMotor();
        }
    }
}
