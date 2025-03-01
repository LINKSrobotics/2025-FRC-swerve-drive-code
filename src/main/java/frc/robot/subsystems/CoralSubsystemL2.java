package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralSubsystemL2 extends SubsystemBase {
    private final SparkMax coralMotorL2;

    public CoralSubsystemL2() {
        coralMotorL2 = new SparkMax(9, MotorType.kBrushless);
    }

    public void setMotorSpeed(double speed) {
        coralMotorL2.set(speed);
    }

    public void stopMotor() {
        coralMotorL2.set(0);
    }
}
