// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Magic;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
          SteerRequestType.MotionMagicExpo); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
          SteerRequestType.MotionMagicExpo); // Use Closed-loop control for drive motors at low speeds

  private final CommandXboxController Pilot = new CommandXboxController(0);

  public final AlgaeSubsystem algae = new AlgaeSubsystem();
  public final CoralSubsystem coral = new CoralSubsystem();
  public final CommandSwerveDrivetrain dt = DrivetrainConstants.createDrivetrain();
  public final VisionSubsystem vision = new VisionSubsystem(dt::addVisionMeasurement, 
  new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0), 
  new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));
  public final CANdleSubsystem candle = new CANdleSubsystem();
  public final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public final Magic magic = new Magic();

  // State Triggers
  // state triggers allow you to access their state indefinitely

  private final Trigger hasCoralTrigger = new Trigger(coral::hasCoral);
  private final Trigger algaeModeEnabled = new Trigger(magic::getAlgaeEnabled);
  private final Superstructure structure = new Superstructure(dt, algae, coral, magic, elevator, candle);

  public RobotContainer() {
    dt.setStateStdDevs(VecBuilder.fill(0.01, 0.01, Math.toRadians(5)));
    configureBindings();
  }

  private void configureBindings() {

    dt.setDefaultCommand(
      // Drivetrain will execute this command periodically
      dt.applyRequest(() -> drive.withVelocityX(Pilot.getLeftY()  * MaxSpeed) // Drive
                                                                                                           // forward
                                                                                                           // with
                                                                                                           // negative
                                                                                                           // Y (up)
          .withVelocityY(-Pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-Pilot.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
      ));

    Pilot.leftTrigger().and(algaeModeEnabled.negate())
          .onTrue(structure.home()); // homing disables an active alignment command

    Pilot.leftTrigger().and(algaeModeEnabled)
          .onTrue(algae.hold()); // a quick press to transition to holding the algae

    Pilot.rightTrigger().and(algaeModeEnabled.negate())
          .onTrue(coral.timedScore()); // Score coral if automation is not working

    Pilot.rightTrigger().and(algaeModeEnabled)
          .onTrue(algae.spit().withTimeout(3)); // Spit alage


    Pilot.leftBumper().and(algaeModeEnabled)
          .whileTrue(structure.intakeAlgaeAndRaiseElevatorOnFalse());

    Pilot.leftBumper().and(algaeModeEnabled.negate())
          .onTrue(structure.AlignAndScore(true));

    Pilot.rightBumper().and(algaeModeEnabled)
          .whileTrue(structure.getGroundAlgae());
    
    Pilot.rightBumper().and(algaeModeEnabled.negate())
          .onTrue(structure.AlignAndScore(false));

    Pilot.y().and(algaeModeEnabled.negate())
          .onTrue(Commands.runOnce(() -> magic.setLevel(4)));
    
    Pilot.x().and(algaeModeEnabled.negate())
          .onTrue(Commands.runOnce(() -> magic.setLevel(3)));

    Pilot.b().and(algaeModeEnabled.negate())
          .onTrue(Commands.runOnce(() -> magic.setLevel(2)));

  
    Pilot.y().and(algaeModeEnabled)
          .whileTrue(structure.getAlgaeFromBotReef());
    
    Pilot.x().and(algaeModeEnabled)
          .whileTrue(structure.getAlgaeFromTopReef());

    Pilot.b().and(algaeModeEnabled)
          .whileTrue(structure.snapToAlgaeAndProcess(
            () -> -Pilot.getLeftX() * MaxSpeed,
            () -> -Pilot.getLeftY() * MaxSpeed, 
            0.7));

    Pilot.povUp().onTrue(structure.prepClimb()); 
// Pilot should note that failed climbs can easily be reset with prepClimb(), as all setpoints command the servo back to 0 (safe)
    Pilot.back().whileTrue(structure.climb());


    Pilot.a()
          .onTrue(Commands.runOnce(() -> magic.toggleAlgaeMode()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
