package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class FlywheelSysIDFactory {
  private final Flywheel flywheel;

  @Getter private final SysIdRoutine routine;

  public FlywheelSysIDFactory(Flywheel flywheel) {
    this.flywheel = flywheel;

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.25),
                Volts.of(3),
                Seconds.of(10),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                flywheel::setVoltage,
                log -> {
                  log.motor("Shooter")
                      .angularVelocity(Rotations.per(Minute).of(flywheel.getVelocity()))
                      .voltage(Volts.of(flywheel.getVolts()))
                      .angularPosition(Rotations.of(flywheel.getPosition()));
                },
                flywheel));
  }
}
