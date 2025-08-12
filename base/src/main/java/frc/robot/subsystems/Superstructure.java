package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CANdleSubsystem.COLOR;

public class Superstructure extends SubsystemBase {
    CommandSwerveDrivetrain dt;
    AlgaeSubsystem algae;
    CoralSubsystem coral;
    Magic magic;
    ElevatorSubsystem elevator;
    CANdleSubsystem candle;

    CommandXboxController pilot;

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Poses/DesiredAPTarget", Pose2d.struct).publish();

    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Poses/APTargetArray", Pose2d.struct).publish();

    private SwerveRequest.FieldCentricFacingAngle headingAwareRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    .withDriveRequestType(DriveRequestType.Velocity)
    .withHeadingPID(4, 0, 0); /* change theese values for your robot */

    private SwerveRequest.FieldCentricFacingAngle snapToAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    .withDriveRequestType(DriveRequestType.Velocity)
    .withHeadingPID(4, 0, 0); /* change theese values for your robot */


    // Intialize superstructure
    public Superstructure(CommandSwerveDrivetrain dt, AlgaeSubsystem alg, CoralSubsystem cor, Magic mag, ElevatorSubsystem elv) {
        this.dt = dt;
        this.algae = alg;
        this.coral = cor;
        this.magic = mag;
        this.elevator = elv;

        pilot = new CommandXboxController(0);
    }

    // Builder Methods

    public Command autopilotToCoralTarget(boolean left) {
        APTarget alignmentTarget = magic.getAPTarget(left, dt.getState().Pose);
        return this.run(() -> {
            Translation2d velocities = getDriveVelocitiesAsTranslation2d();
            Pose2d currentPose = dt.getState().Pose;

            APResult output = Constants.AutopilotConstants.kAutopilot.calculate(currentPose, velocities, alignmentTarget);

            dt.setControl(headingAwareRequest
            .withVelocityX(output.vx())
            .withVelocityY(output.vy())
            .withTargetDirection(output.targetAngle()));
        }).until(() -> 
            Constants.AutopilotConstants.kAutopilot.atTarget(dt.getState().Pose, alignmentTarget))
        .andThen(
            runOnce(() -> dt.stopWithX()),
            candle.strobeAnimation(COLOR.PURPLE).andThen(new WaitCommand(2)).andThen(candle.defaultcmd())
        );
    }

    public Command handleAutoScore() {
        int lev = magic.getLevel();
        if (magic.getAutomation() == 1) {
            return Commands.none();
        } else {
            if (lev == 4) {
                return coral.timedScore();
            } else {
                return coral.timedLowScore();
            }
        }
    }

    public Command autopilotToTarget(APTarget target) {
        return this.run(() -> {
            Translation2d velocities = getDriveVelocitiesAsTranslation2d();
            Pose2d currentPose = dt.getState().Pose;

            APResult output = Constants.AutopilotConstants.kAutopilot.calculate(currentPose, velocities, target);

            dt.setControl(headingAwareRequest
            .withVelocityX(output.vx())
            .withVelocityY(output.vy())
            .withTargetDirection(output.targetAngle()));
        }).until(() -> 
            Constants.AutopilotConstants.kAutopilot.atTarget(dt.getState().Pose, target))
        .andThen(
            runOnce(() -> dt.stopWithX()),
            candle.strobeAnimation(COLOR.BLUE).andThen(new WaitCommand(2)).andThen(candle.defaultcmd())
        );
    }

    public Command snapToProcessor(DoubleSupplier joystickX, DoubleSupplier joystickY, double slowFactor) {
        Rotation2d targetAngle = Constants.isBlueAlliance() 
            ? Constants.FieldConstants.BLUE_PROCESSOR_ANGLE
            : Constants.FieldConstants.RED_PROCESSOR_ANGLE;
        int negativeMulti = Constants.isBlueAlliance() ? 1 : -1;
            return run(() -> {
            dt.setControl(
                snapToAngle
                .withVelocityX(negativeMulti * joystickX.getAsDouble() * slowFactor)
                .withVelocityY(negativeMulti * joystickY.getAsDouble() * slowFactor)
                .withTargetDirection(targetAngle)
            );
        });
    }

    public Command snapToBarge(DoubleSupplier joystickX, DoubleSupplier joystickY, double slowFactor) {
        Rotation2d targetAngle = Constants.isBlueAlliance() 
            ? Constants.FieldConstants.BLUE_CLIMB_ANGLE
            : Constants.FieldConstants.RED_CLIMB_ANGLE;
        int negativeMulti = Constants.isBlueAlliance() ? 1 : -1;
        return run(() -> {
            dt.setControl(
                snapToAngle
                .withVelocityX(negativeMulti * joystickX.getAsDouble() * slowFactor)
                .withVelocityY(negativeMulti * joystickY.getAsDouble() * slowFactor)
                .withTargetDirection(targetAngle)
            );
        });
    }

    // Teleop methods


    // A fully integrated method that for L2 and L3 scoring that will align, raise the elevator, score, and zero. Otherwise,
    // it will align, score, and rely on operator to zero.
    public Command AlignAndScore(boolean left) {
        int lev = magic.getLevel();
        Command setpoint = Commands.none();

        switch (lev) {
            case 4 -> setpoint = elevator.goToSetpointL4();
            case 3 -> setpoint = elevator.goToSetpointL3();
            case 2 -> setpoint = elevator.goToSetpointL2();
        }

        if (lev == 3 || lev == 2) {
             return Commands.parallel(
                autopilotToCoralTarget(left),
                setpoint
            ).andThen(
                handleAutoScore()
            ).andThen(elevator.zero().alongWith(candle.defaultcmd()));
        } else if (lev == 4) {
            return Commands.parallel(
                autopilotToCoralTarget(left),
                setpoint
            ).andThen(
                handleAutoScore().alongWith(candle.defaultcmd())
            );
        } else {
            DogLog.log("Failed to align. Level not set or invalid.", AlertType.kWarning);
            return Commands.none();
        }
    }

    public Command home() {
        return Commands.parallel(
            elevator.zero(),
            coral.passive(),
            algae.stop().withTimeout(0.1), // We only need to command the algae motor to stop momentarily
            runOnce(() -> dt.stop()) // use to stop drivetrain and cancel an Autopilot action
        );
    }

    // A utility command for intaking algae, and once a control is false, the elevator will raise slightly to allow the algae to be released.
    public Command intakeAlgaeAndRaiseElevatorOnFalse() {
        return runEnd(() -> {
            algae.intake();
        }, () -> {
            elevator.bumpUp(.25);
            algae.hold();
            candle.strobeAnimation(COLOR.PURPLE).andThen(new WaitCommand(2)).andThen(candle.defaultcmd());
        });
    }


    // a command to start the algae intake once we reach the setpoint for the bottom algae
    public Command getAlgaeFromBotReef() {
        return Commands.sequence(
            elevator.goToSetpointBotAlgae(),
            algae.intake()
        );
    }

    // a command to start the algae intake once we reach the setpoint for the top algae
    public Command getAlgaeFromTopReef() {
        return Commands.sequence(
            elevator.goToSetpointTopAlgae(),
            algae.intake()
        );
    }

    // a command to start the algae intake once we reach the setpoint for the ground algae
    public Command getGroundAlgae() {
        return Commands.sequence(
            elevator.goToSetpointGroundAlgae(),
            algae.intake()
        );
    }

    // A command that will, while a binding is held, hold algae and snap the drivetrain angle to the processor.
    // When the binding is released, the algae will be processed. Experimental feature - the elevator will come up as we
    // spit to help the algae "fall" in an inwards arc towards the processor.

    public Command snapToAlgaeAndProcess(DoubleSupplier jx, DoubleSupplier jy, double slowFactor) {
        return runEnd(() -> {
            algae.hold();
            snapToProcessor(jx, jy, slowFactor);
            candle.setColor(COLOR.GREEN);
        }, () -> {
            runOnce(() -> elevator.bumpUp(.1));
            algae.spit()
            .withTimeout(3)
            .andThen(algae.stop().alongWith(candle.defaultcmd()));
        });
    }

    public Command prepClimb(DoubleSupplier jx, DoubleSupplier jy, double slowFactor) {
        return Commands.sequence(
            algae.stop(),
            coral.stop(),
            candle.setColor(COLOR.ORANGE)
        ).andThen(
            elevator.goToSetpointL2()
        );
    }

    public Command climb() {
        return Commands.parallel(
            elevator.climb(),
            candle.strobeAnimation(COLOR.GREEN)
        );
    }


    // Utilities

    private Translation2d getDriveVelocitiesAsTranslation2d() {
        double vx = dt.getState().Speeds.vxMetersPerSecond;
        double vy = dt.getState().Speeds.vyMetersPerSecond;
        Rotation2d theta = dt.getState().Pose.getRotation();

        return new Translation2d(vx, vy).rotateBy(theta);
    }

    public CommandXboxController getPilot() {
        return pilot;
    }


    // get all subsystems that are useful during binding

    public ElevatorSubsystem getElevator() {
        return elevator;
    }

    public CoralSubsystem getCoralSubsystem() {
        return coral;
    }

    public AlgaeSubsystem getAlgaeSubsystem() {
        return algae;
    }

    public Magic getMagic() {
        return magic;
    }


    @Override
    public void periodic() {

        Pose2d currentAlignmentTarget = magic.getCurrentAPTarget().getReference();

        publisher.set(currentAlignmentTarget);
        arrayPublisher.set(new Pose2d[] {currentAlignmentTarget});
    }
}
