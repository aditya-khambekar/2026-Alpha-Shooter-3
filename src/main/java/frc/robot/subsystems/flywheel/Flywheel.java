package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import lombok.Getter;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs;

  @Getter private final FlywheelSysIDFactory sysIDFactory;

  LoggedTunableNumber desiredMotorVelocity = new LoggedTunableNumber("Motor Velocity RPM");

  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.inputs = new FlywheelIOInputsAutoLogged();

    this.sysIDFactory = new FlywheelSysIDFactory(this);

    desiredMotorVelocity.initDefault(0);
  }

  public void setVoltage(Voltage volts) {
    System.out.println("Flywheel Set Voltage " + volts.in(Volts));
    io.setMotorVoltage(volts);
  }

  public void setVelocitySetpoint(AngularVelocity velocity) {
    io.setVelocitySetpoint(velocity);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public double getVolts() {
    return io.get() * RobotController.getBatteryVoltage();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public double getPosition() {
    return io.getPosition();
  }

  public Command runVelocity() {
    return this.runEnd(
        () -> {
          setVelocitySetpoint(Rotations.per(Minute).of(desiredMotorVelocity.get()));
        },
        () -> {
          setVoltage(Volts.zero());
        });
  }

  public Command runVelocity(AngularVelocity v) {
    return this.runEnd(
        () -> {
          setVelocitySetpoint(v);
        },
        () -> {
          setVoltage(Volts.zero());
        });
  }

  public Command runVelocityWPI() {
    return new VelocityWPI();
  }

  public class VelocityWPI extends Command {
    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(0.074548, 0.10976, 0.044959);
    private final ProfiledPIDController controller =
        new ProfiledPIDController(
            0.43619, 0, 0, new Constraints(300, 600) // acceleration, jerk
            );

    @Override
    public void execute() {
      var goal = Flywheel.this.desiredMotorVelocity.get() / 60;
      controller.setGoal(goal);
      var desiredMotorVelocity = controller.getSetpoint().position;
      var desiredMotorAcceleration = controller.getSetpoint().velocity;
      Flywheel.this.setVoltage(
          Volts.of(
              feedforward.calculate(desiredMotorVelocity, desiredMotorAcceleration)
                  + controller.calculate(Flywheel.this.getVelocity())));
    }
  }
}
