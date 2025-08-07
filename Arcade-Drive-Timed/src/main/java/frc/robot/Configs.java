package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Centralized configuration class to store motor controller settings.
 * Keeps all motor configs organized and reusable.
 */
public final class Configs {

    public static final class Drivetrain {

        // Config objects for left and right SparkMax motors
        public static final SparkMaxConfig leftConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightConfig = new SparkMaxConfig();

        static {
            // Basic motor settings common to both left and right motors
            // idleMode: Sets motor behavior when no power is applied (Brake mode here)
            // smartCurrentLimit: Limits current draw to prevent damage or brownouts
            // voltageCompensation: Helps maintain consistent performance with battery voltage changes
            leftConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12);

            rightConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12);

            // Closed-loop (PID) controller setup for velocity control using the integrated encoder
            // feedbackSensor: Using the built-in Neo encoder as the sensor source
            // pid(p, i, d): PID gains to tune velocity response
            // velocityFF: Feedforward gain to help maintain target velocity
            // outputRange: Limits output from -1 to 1 (full reverse to full forward)
            leftConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0002, 0, 0)
                .velocityFF(0.000015)
                .outputRange(-1, 1);

            rightConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0002, 0, 0)
                .velocityFF(0.000015)
                .outputRange(-1, 1);
        }
    }
}
