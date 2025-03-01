package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax coralMotorL1;
    private final SparkMax coralMotorL2;


    public CoralSubsystem() {
        coralMotorL1 = new SparkMax(10, MotorType.kBrushless);
        coralMotorL2 = new SparkMax(9, MotorType.kBrushless);

    }

    public void setMotorSpeed(double speed) {
        coralMotorL1.set(speed);
        coralMotorL2.set(speed);
    }

    public void stopMotor() {
        coralMotorL1.set(0);
        coralMotorL2.set(0);
    }
}
