package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

  TalonSRX coral = new TalonSRX(12);
  private AnalogInput IR = new AnalogInput(1);

  // default values 
  double irSensorThreshold = 1.8;
  double defaultMax = 1.0;
  double defaultLow = 0.5;
  double defaultScoreTime = 0.4;

  // Tunables
  private final DoubleSubscriber tunableOutputHigh = DogLog.tunable("Coral/HighOut", 1.0, newL4 -> {
    defaultMax = newL4;
  });

  private final DoubleSubscriber tunableOutputLow = DogLog.tunable("Coral/LowOut", 0.5, newMidlevel -> {
    defaultLow = newMidlevel;
  });

  private final DoubleSubscriber tunableTimedThreshold = DogLog.tunable("Coral/ScoreTimer", 0.4, newTime -> {
    defaultScoreTime = newTime;
  });

  private final DoubleSubscriber tunableSensorThreshold = DogLog.tunable("Coral/TunableThreshold", 1.8, newThreshold -> {
    irSensorThreshold = newThreshold;
  });


  public CoralSubsystem() {
    coral.configFactoryDefault();
    IR.setOversampleBits(4);
    IR.setAverageBits(4);
  }

  public void periodic() {
    DogLog.log("Coral/OutputPerc", coral.getMotorOutputPercent());
    DogLog.log("Coral/OutputVolt", coral.getMotorOutputVoltage());
    DogLog.log("Coral/Temp", coral.getTemperature());
  }

  // Default Commands
  public Command run(double percent) {
    return run(() -> {
        coral.set(TalonSRXControlMode.PercentOutput, percent);
    });
  }

  // Timed scoring commands
  public Command timedScore() {
    return run(tunableOutputHigh.get()).withTimeout(tunableTimedThreshold.get());
  }

  public Command timedLowScore() {
    return run(tunableOutputLow.get()).withTimeout(tunableTimedThreshold.get());
  }

  // Helper methods
  private int integ = 0;
  public boolean hasCoral() {
    if (IR.getAverageVoltage() > tunableSensorThreshold.get()) {
     integ++;
    } else {
     integ = 0;
    }
    return integ >=5;
 }
}
