// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class Configs {
     public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class CoralSubsystem {
        public static final SparkMaxConfig leftCoralMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightCoralMotorConfig = new SparkMaxConfig();

        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                /*double coralFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                        / ModuleConstants.kDrivingMotorReduction;
                double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
                */
                leftCoralMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);
                /*leftCoralMotorConfig.encoder
                        .positionConversionFactor(coralFactor) // meters
                        .velocityConversionFactor(coralFactor / 60.0); // meters per second
                */
    
                rightCoralMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);
                /*rightCoralMotorConfig.encoder
                        .positionConversionFactor(coralFactor) // meters
                        .velocityConversionFactor(coralFactor / 60.0); // meters per second
                */
            }
    }

    public static final class AlageSubsystem {
        public static final SparkMaxConfig highAlgaeMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig lowAlgaeMotorConfig = new SparkMaxConfig();

        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                /*double algaeFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                        / ModuleConstants.kDrivingMotorReduction;
                double algaeVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
                */
                highAlgaeMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);
                /*leftAlgaeMotorConfig.encoder
                        .positionConversionFactor(algaeFactor) // meters
                        .velocityConversionFactor(algaeFactor / 60.0); // meters per second
                */
                lowAlgaeMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);
                /*rightAlgaeMotorConfig.encoder
                        .positionConversionFactor(algaeFactor) // meters
                        .velocityConversionFactor(algaeFactor / 60.0); // meters per second
                */
          }
    }

}
