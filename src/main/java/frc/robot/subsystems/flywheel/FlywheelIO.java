package frc.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public MutAngularVelocity motorVelocity;
        public MutCurrent motorCurrent;
        public MutTemperature motorTemperature;
    }

    public default void setMotorVoltage(Voltage volts) {}

    public default void setVelocitySetpoint(AngularVelocity angularVelocity) {}

    public default void updateInputs(FlywheelIOInputs inputs) {}
}
