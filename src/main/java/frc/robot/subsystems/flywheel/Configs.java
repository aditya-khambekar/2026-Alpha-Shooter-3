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
        .apply(new EncoderConfig().velocityConversionFactor(1 / 60.0));
    leaderConfig.closedLoop.feedForward.sva(0.074548, 0.10976, 0.044959);
    leaderConfig
        .closedLoop
        .p(0.0090597)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxMotion
        .maxAcceleration(300);

    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(90, 90)
        .follow(Constants.IDs.SHOOTER_LEADER, true);
  }
}
