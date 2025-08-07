package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ElevatorSetpointConfigs;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX master;
    private TalonFX follower; 
    private Servo servo;
    private DigitalInput limit;

    // Tunable Coral values
    private final DoubleSubscriber elevatorZeroTimeout = DogLog.tunable("Elevator/ZeroTimeout", 3.0);
    private final DoubleSubscriber elevatorMaximumHeight = DogLog.tunable("Elevator/MaxHeight", 8.3);
    private final DoubleSubscriber elevatorL4 = DogLog.tunable("Elevator/L4", 8.25);
    private final DoubleSubscriber elevatorL3 = DogLog.tunable("Elevator/L3", 4.5);
    private final DoubleSubscriber elevatorL2 = DogLog.tunable("Elevator/L2", 2.08);
    private final DoubleSubscriber elevatorHome = DogLog.tunable("Elevator/Home", 0.0);

    // Tunable Algae values
    private final DoubleSubscriber elevatorAlgaeTop = DogLog.tunable("Elevator/AlgaeTop", 7.7);
    private final DoubleSubscriber elevatorAlgaeBot = DogLog.tunable("Elevator/AlgaeBot", 4.9);
    private final DoubleSubscriber elevatorAlgaeProcessor = DogLog.tunable("Elevator/AlgaeProcessor", 2.2);
    private final DoubleSubscriber elevatorAlgaeGround = DogLog.tunable("Elevator/AlgaeGround", 0.72);

    // Tunable Voltages 
    private final DoubleSubscriber elevatorClimbVoltage = DogLog.tunable("Elevator/ClimbingVoltage", -5.0);
    private final DoubleSubscriber elevatorZeroVoltage = DogLog.tunable("Elevator/ZeroVoltage", -6.0);

    // CTRE MotorOutput Requests
    VoltageOut voltageRequest;
    MotionMagicVoltage motionRequest;


    public ElevatorSubsystem() {
        DogLog.log("ElevatorSubsystem/Ready", false);
        master = new TalonFX(Constants.CAN_IDS.ELEVATOR.ELEVATOR_MASTER, "CAN-2");
        follower = new TalonFX(Constants.CAN_IDS.ELEVATOR.ELEVATOR_FOLLOWER, "CAN-2");
        servo = new Servo(Constants.PWM_IDS.SERVO);

        voltageRequest = new VoltageOut(0);
        motionRequest = new MotionMagicVoltage(0);

        motionRequest.EnableFOC = true;
        voltageRequest.EnableFOC = true;

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ElevatorSetpointConfigs.ELEVATOR_FORWARD_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ElevatorSetpointConfigs.ELEVATOR_REVERSE_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // TESTING ONLY
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TESTING ONLY
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimit = 54;
        masterConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        masterConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        masterConfig.CurrentLimits.StatorCurrentLimit = 120;
        masterConfig.Feedback.SensorToMechanismRatio = 12;
        masterConfig.Voltage.PeakForwardVoltage = 16;
        masterConfig.Voltage.PeakReverseVoltage = -16;

        masterConfig.Slot0.kP = 18;
        masterConfig.Slot0.kI = 2;
        masterConfig.Slot0.kD = 2;
        masterConfig.Slot0.kG = 0.3;
        masterConfig.Slot0.kS = 0.1;
        masterConfig.MotionMagic.MotionMagicCruiseVelocity = 45;
        masterConfig.MotionMagic.MotionMagicAcceleration = 110;

        masterConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        master.getConfigurator().apply(masterConfig);
        follower.getConfigurator().apply(masterConfig);
        
        follower.setControl(new Follower(master.getDeviceID(), true));
        master.setPosition(0.0);
        follower.setPosition(0.0);

        DogLog.log("ElevatorSubsystem/Ready", true);
    }

    // Commands

    public Command zero() {
        return runEnd(() -> {
            master.setControl(voltageRequest.withOutput(elevatorZeroVoltage.get()));
        }, () -> {
            DogLog.log("Elevator/CurrentSetpointReached", "HOME");
            master.set(0);
        })
        .until(() -> !limit.get())
        .withTimeout(elevatorZeroTimeout.get());
    }

    public Command testElevator(double voltage) {
        return runEnd(() -> {
            servo.set(0);
            master.setControl(voltageRequest.withOutput(voltage));
        }, () -> {
            master.set(0);
        });
    }

    public Command climb() {
        return runEnd(() -> {
            DogLog.log("Elevator/ClimbActive", true);
            servo.set(0.5);
            master.setControl(voltageRequest.withOutput(elevatorClimbVoltage.get()));
        }, () -> {
            master.set(0);
        }).until(() -> !limit.get());
    }

    public Command bumpUp(double bumpUpValue) {
        double targetedBumpUp  = getPosition() + bumpUpValue;
        return runEnd(() -> {
            servo.set(0);
            master.setControl(motionRequest.withPosition(targetedBumpUp));
        }, () -> {
            master.set(0);
        }).until(() -> isNearPosition(targetedBumpUp));
   
    }


    // Setpoint commands will terminate after a cooldown. This is to prevent a firmware auto-disable if the elevator is commanded for too long.
    // Furthermore, robot elevator actions should not take longer than 5-8 seconds. 

    public Command goToSetpointL4() {
        return runEnd(() -> {
            servo.set(0);
            master.setControl(motionRequest.withPosition(elevatorL4.get()));
        }, () -> {
            master.setControl(motionRequest.withPosition(master.getPosition().getValueAsDouble()));
            DogLog.log("Elevator/CurrentSetpointReached", "L4");
        }).
        until(
            () -> isNearPosition(elevatorL4.get()));
    }

    public Command goToSetpointL3() {
        return runEnd(() -> {
            servo.set(0);
            master.setControl(motionRequest.withPosition(elevatorL3.get()));
        }, () -> {
            master.setControl(motionRequest.withPosition(master.getPosition().getValueAsDouble()));
            DogLog.log("Elevator/CurrentSetpointReached", "L3");
        }).
        until(
            () -> isNearPosition(elevatorL3.get()));
    }

    public Command goToSetpointL2() {
        return runEnd(() -> {
            servo.set(0);
            master.setControl(motionRequest.withPosition(elevatorL2.get()));
        }, () -> {
            master.setControl(motionRequest.withPosition(master.getPosition().getValueAsDouble()));
            DogLog.log("Elevator/CurrentSetpointReached", "L2");
        }).
        until(
            () -> isNearPosition(elevatorL2.get()));
    }

    public Command goToSetpointTopAlgae() {
        return runEnd(() -> {
            servo.set(0);
            master.setControl(motionRequest.withPosition(elevatorAlgaeTop.get()));
        }, () -> {
            master.setControl(motionRequest.withPosition(master.getPosition().getValueAsDouble()));
            DogLog.log("Elevator/CurrentSetpointReached", "ALG_TOP");
        }).
        until(
            () -> isNearPosition(elevatorAlgaeTop.get()));
    }

    public Command goToSetpointBotAlgae() {
        return runEnd(() -> {
            servo.set(0);
            master.setControl(motionRequest.withPosition(elevatorAlgaeBot.get()));
        }, () -> {
            master.setControl(motionRequest.withPosition(master.getPosition().getValueAsDouble()));
            DogLog.log("Elevator/CurrentSetpointReached", "ALG_BOT");
        }).
        until(
            () -> isNearPosition(elevatorAlgaeBot.get()));
    }

    public Command goToSetpointGroundAlgae() {
        return runEnd(() -> {
            servo.set(0);
            master.setControl(motionRequest.withPosition(elevatorAlgaeGround.get()));
        }, () -> {
            master.setControl(motionRequest.withPosition(master.getPosition().getValueAsDouble()));
            DogLog.log("Elevator/CurrentSetpointReached", "ALG_GRO");
        }).
        until(
            () -> isNearPosition(elevatorAlgaeBot.get()));
    }

    public Command goToSetpointProcessor() {
        return runEnd(() -> {
            servo.set(0);
            master.setControl(motionRequest.withPosition(elevatorAlgaeProcessor.get()));
        }, () -> {
            master.setControl(motionRequest.withPosition(master.getPosition().getValueAsDouble()));
            DogLog.log("Elevator/CurrentSetpointReached", "ALG_PRO");
        }).
        until(
            () -> isNearPosition(elevatorAlgaeBot.get()));
    }

    // Utilities
    public boolean isNearPosition(double rotations) {
        return Math.abs(master.getPosition().getValueAsDouble() - rotations) < ElevatorSetpointConfigs.ELEVATOR_DEADZONE_DIST;
    }

    public double getPosition() {
        return master.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        DogLog.log("Elevator/Master/Position", master.getPosition().getValueAsDouble());
        DogLog.log("Elevator/Master/Velocity", master.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/Master/Output", master.get());
        DogLog.log("Elevator/Master/OutputCurrent", master.getStatorCurrent().getValueAsDouble());

        DogLog.log("Elevator/Master/DeviceTemp", master.getDeviceTemp().getValueAsDouble());
        DogLog.log("Elevator/Follower/DeviceTemp", follower.getDeviceTemp().getValueAsDouble());
        DogLog.log("Elevator/Limit", !limit.get());

        if (master.getDeviceTemp().getValueAsDouble() > 60.0 || follower.getDeviceTemp().getValueAsDouble() > 60.0) {
            DogLog.logFault("HOT CAUTION - Elevator", AlertType.kWarning);
        }

        if (!limit.get() && (master.getPosition().getValueAsDouble() != 0)) {
            master.setPosition(0);
          }
    }
}
