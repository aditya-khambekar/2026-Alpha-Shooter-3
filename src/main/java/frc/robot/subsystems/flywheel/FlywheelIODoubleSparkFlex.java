package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.LoggedTunableNumber;

public class FlywheelIODoubleSparkFlex implements FlywheelIO {
    private final SparkFlex leader;
    private final SparkFlex follower;

    public FlywheelIODoubleSparkFlex(int leaderID, int followerID) {
        leader = new SparkFlex(leaderID, MotorType.kBrushless);
        follower = new SparkFlex(followerID, MotorType.kBrushless);

        leader.configure(Configs.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        follower.configure(Configs.followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel kP");
        kP.initDefault(leader.configAccessor.closedLoop.getP());
        LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel kI");
        kI.initDefault(leader.configAccessor.closedLoop.getI());
        LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel kD");
        kD.initDefault(leader.configAccessor.closedLoop.getD());

        LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Flywheel Cruise Velocity");
        LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("Flywheel Max Acceleration");

        LoggedTunableNumber.ifChanged(
                this.hashCode(),
                pid -> {
                    Configs.leaderConfig.closedLoop.pid(pid[0], pid[1], pid[2]);
                    leader.configure(
                            Configs.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
                },
                kP,
                kI,
                kD);

        LoggedTunableNumber.ifChanged(
                this.hashCode(),
                va -> {
                    Configs.leaderConfig.closedLoop.maxMotion.cruiseVelocity(va[0]);
                    Configs.leaderConfig.closedLoop.maxMotion.maxAcceleration(va[1]);
                    leader.configure(
                            Configs.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
                },
                maxVelocity,
                maxAcceleration);
    }

    @Override
    public void setMotorVoltage(Voltage volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void setVelocitySetpoint(AngularVelocity velocity) {
        leader.getClosedLoopController()
                .setSetpoint(velocity.in(Rotations.per(Minute)), ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.motorVelocity.mut_replace(leader.getEncoder().getVelocity(), Rotations.per(Minute));
        inputs.motorCurrent.mut_replace(leader.getOutputCurrent(), Amps);
        inputs.motorTemperature.mut_replace(leader.getMotorTemperature(), Celsius);
    }
}
