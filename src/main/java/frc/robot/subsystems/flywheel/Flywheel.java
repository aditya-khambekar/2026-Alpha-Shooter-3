package frc.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs;

    @Getter
    private final FlywheelSysIDFactory sysIDFactory;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        this.inputs = new FlywheelIOInputsAutoLogged();

        this.sysIDFactory = new FlywheelSysIDFactory(this);
    }

    public void setVoltage(Voltage volts) {
        io.setMotorVoltage(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
