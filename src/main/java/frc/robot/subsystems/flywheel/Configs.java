package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class Configs {
  public static SparkFlexConfig leaderConfig = new SparkFlexConfig();
  public static SparkFlexConfig followerConfig = new SparkFlexConfig();

  static {
    leaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(90, 90)
        .apply(new EncoderConfig().velocityConversionFactor(1 / 60));
    leaderConfig.closedLoop.feedForward.sva(0.13955, 0.11141, 0.036289);
    leaderConfig
        .closedLoop
        .p(9.982E-05)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxMotion
        .cruiseVelocity(6000)
        .maxAcceleration(3000)
        .allowedProfileError(1);

    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(90, 90)
        .follow(Constants.IDs.SHOOTER_LEADER, true);
  }
}
