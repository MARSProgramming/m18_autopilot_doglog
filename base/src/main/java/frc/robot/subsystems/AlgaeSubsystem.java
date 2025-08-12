package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

  SparkMax algae = new SparkMax(12, MotorType.kBrushless);

  // default values 


  // Tunables
  private final DoubleSubscriber tunableStallThreshold = DogLog.tunable("Algae/StallDetection", 30.0);

  private final DoubleSubscriber tunableSpit = DogLog.tunable("Algae/SpitPercent", 1.0);

  private final DoubleSubscriber tunableIntake = DogLog.tunable("Algae/IntakePercent", -1.0);

  private final DoubleSubscriber tunableHold = DogLog.tunable("Algae/HoldPercent", -0.25);  

  public AlgaeSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    config.inverted(true);
    algae.configure(config, null, null);
  }

  @Override
  public void periodic() {
    DogLog.log("Algae/OutputCurrent", algae.getOutputCurrent());
    DogLog.log("Algae/NominalVoltage", algae.getBusVoltage());
    DogLog.log("Algae/Temp", algae.getMotorTemperature());

    if (algae.getMotorTemperature() > 60.0) {
        DogLog.logFault("HOT CAUTION - Algae", AlertType.kWarning);
    }
  }

  // Default Commands
  public Command intake() {
    return run(() -> {
        algae.set(tunableIntake.get());
    });
  }

  public Command spit() {
    return run(() -> {
        algae.set(tunableSpit.get());
    });
  }

  
  public Command hold() {
    return run(() -> {
        algae.set(tunableHold.get());
    });
  }

  public Command stop() {
    return runOnce(() -> {
      algae.set(0);
    });
  }

  // State-aware commands
  public Command intakeUntilStalled() {
    return run(() -> {
        intake();
    }).until(() -> algae.getOutputCurrent() > tunableStallThreshold.get());
  }
}
