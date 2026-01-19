package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.FeedbackSensor;
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
                .closedLoop
                .pid(0, 0, 0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                .cruiseVelocity(6000)
                .maxAcceleration(3000);

        followerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(90, 90).follow(Constants.IDs.SHOOTER_LEADER);
    }
}
