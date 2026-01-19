package frc.robot.subsystems.flywheel;

import javax.annotation.processing.Generated;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;

public class FlywheelSysIDFactory {
    private final Flywheel flywheel;
    @Getter
    private final SysIdRoutine routine;

    public FlywheelSysIDFactory(Flywheel flywheel) {
        this.flywheel = flywheel;

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(flywheel::setVoltage, null, flywheel));
    }
}
