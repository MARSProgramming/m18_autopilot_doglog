package frc.robot.bindings;

import frc.robot.subsystems.Superstructure;

public class ControllerBinding {
    public Superstructure supr;

    public ControllerBinding(Superstructure superstructure) {
        this.supr = superstructure;
    }

    public void bind() {
        // Bind the controller buttons to the superstructure commands, dependent on algae state
        
        // update: we will have to create named commands like getXButtonCommand() to bind to the controller buttons
        
    }
}
