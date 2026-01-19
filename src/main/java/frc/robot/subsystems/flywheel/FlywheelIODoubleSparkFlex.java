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

public class FlywheelIODoubleSparkFlex implements FlywheelIO {
    private final SparkFlex leader;
    private final SparkFlex follower;

    public FlywheelIODoubleSparkFlex(int leaderID, int followerID) {
        leader = new SparkFlex(leaderID, MotorType.kBrushless);
        follower = new SparkFlex(followerID, MotorType.kBrushless);

        leader.configure(Configs.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        follower.configure(Configs.followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
