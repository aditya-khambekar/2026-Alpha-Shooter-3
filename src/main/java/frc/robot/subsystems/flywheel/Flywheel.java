package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs;

    public Flywheel(FlywheelIO io){
        this.io = io;
        this.inputs = new FlywheelIOInputsAutoLogged();
    } 

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
