package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superbinder extends SubsystemBase {
    private Superstructure structure;

    public Superbinder(Superstructure struct) {
        structure = struct;
    }

    // Named commands for every button/input we want to use on the Xbox Controller.

    public Command leftBumperCommand() {
        return new ConditionalCommand(
            structure.AlignAndScore(true), 
            structure.intakeAlgaeAndRaiseElevatorOnFalse(), 
            structure.getMagic()::getAlgaeEnabled);
    }




}
