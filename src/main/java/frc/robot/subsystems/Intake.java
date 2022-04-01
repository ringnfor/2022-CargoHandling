// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  public enum ArmState {OPENED, CLOSED};

  private CANSparkMax m_intakeWheels = new CANSparkMax(CAN_IDs.intakeWheels_ID, MotorType.kBrushless);
  private CANSparkMax m_intakeArm = new CANSparkMax(CAN_IDs.intakeArm_ID, MotorType.kBrushless);
  private ArmState m_armState = ArmState.CLOSED;
  
  /** Creates a new Intake. */
  public Intake() {
    m_intakeWheels.restoreFactoryDefaults();
    m_intakeArm.restoreFactoryDefaults();
    m_intakeWheels.setIdleMode(IdleMode.kCoast);
    m_intakeArm.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }



  public void pullInCargo() {
    m_intakeWheels.set(IntakeConstants.wheelMotorSpeed);
  }

  public void pushOutCargo() {
    m_intakeWheels.set(-IntakeConstants.wheelMotorSpeed);
  }

  public void stopIntakeWheels() {
    m_intakeWheels.set(0.0);
  }
}
