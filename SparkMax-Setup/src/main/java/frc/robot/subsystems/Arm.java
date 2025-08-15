// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private SparkMax armMotor = new SparkMax(55, MotorType.kBrushless);
  private RelativeEncoder armEncoder = armMotor.getEncoder();
  private static final double ARM_SPEED = 0.2;

  public Arm() {

    armMotor.configure(
      Configs.ArmSubsystem.armConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    armEncoder.setPosition(0);

  }

    /** Drive the arm forward at a fixed speed */
    public void driveForward() {
      armMotor.set(ARM_SPEED);
    }

    /** Drive the arm backward at a fixed speed */
    public void driveBackward() {
      armMotor.set(-ARM_SPEED);
    }

    /** Stop arm movement */
    public void stop() {
      armMotor.set(0);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
      SmartDashboard.putNumber("Arm Velocity", armEncoder.getVelocity());
    }
}

