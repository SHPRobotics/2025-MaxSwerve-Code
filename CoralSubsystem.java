// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
  private final SparkMax m_leftCoralMotor, m_rightCoralMotor;

  //private final RelativeEncoder m_leftCoralEncoder, m_rightCoralEncoder;
  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    // motors
    m_leftCoralMotor = new SparkMax(CoralConstants.kLeftCoralCANId, MotorType.kBrushless);
    m_rightCoralMotor = new SparkMax(CoralConstants.kRightCoralCANId, MotorType.kBrushless);

    m_leftCoralMotor.configure(Configs.CoralSubsystem.leftCoralMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightCoralMotor.configure(Configs.CoralSubsystem.rightCoralMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     //SmartDashboard.putNumber("Coral_Left RPM", m_leftCoralEncoder.getVelocity());
   // SmartDashboard.putNumber("Coral_Right RPM", m_rightCoralEncoder.getVelocity());
  }

  public void ShootCoralOut(){
    m_leftCoralMotor.set(CoralConstants.kCoralSpeedOut);
    m_rightCoralMotor.set(-CoralConstants.kCoralSpeedOut);
  }

  public void FeedCoralIn(){
    m_leftCoralMotor.set(-CoralConstants.kCoralSpeedIn);
    m_rightCoralMotor.set(CoralConstants.kCoralSpeedIn);
  }

  public void CoralStop() {
    m_leftCoralMotor.set(0);
    m_rightCoralMotor.set(0);
  }

  // same as ShootCoralOut() except is a Command class
  public Command ShootCoralOutCmd() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
                m_leftCoralMotor.set(CoralConstants.kCoralSpeedOut);
                m_rightCoralMotor.set(-CoralConstants.kCoralSpeedOut);
              }
    );
  }
  // same as shooterFeedNoteIn() except it is a Command class
   public Command FeedCoralInCmd() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
                m_leftCoralMotor.set(-CoralConstants.kCoralSpeedIn);
                m_rightCoralMotor.set(CoralConstants.kCoralSpeedIn);
              }
    );
  }

  // same as shooterStop() except it is a Command class
  public Command CoralStopCmd() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
                m_leftCoralMotor.set(0);
                m_rightCoralMotor.set(0);
              }
    );
  }
}
