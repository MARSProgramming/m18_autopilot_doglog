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

  // Tunables
  private final DoubleSubscriber tunableOutputHigh = DogLog.tunable("Coral/HighOut", 1.0);
  private final DoubleSubscriber tunableOutputLow = DogLog.tunable("Coral/LowOut", 0.5);
  private final DoubleSubscriber tunableDefaultOutput = DogLog.tunable("Coral/Default", -0.25);
  private final DoubleSubscriber tunableTimedThreshold = DogLog.tunable("Coral/ScoreTimer", 0.4);
  private final DoubleSubscriber tunableSensorThreshold = DogLog.tunable("Coral/TunableThreshold", 1.8);


  public CoralSubsystem() {
    coral.configFactoryDefault();
    IR.setOversampleBits(4);
    IR.setAverageBits(4);
  }

  @Override
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

  public Command testCoralMotor() {
    return runEnd(() -> {
        coral.set(TalonSRXControlMode.PercentOutput, tunableOutputHigh.get());
    }, () -> {
        coral.set(TalonSRXControlMode.PercentOutput, 0);
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
