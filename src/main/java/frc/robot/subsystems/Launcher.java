// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
  private CANSparkMax m_launcher1 = new CANSparkMax(CAN_IDs.launcher1_ID, MotorType.kBrushless);
  private CANSparkMax m_launcher2 = new CANSparkMax(CAN_IDs.launcher2_ID, MotorType.kBrushless);

  /** Creates a new Launcher. */
  public Launcher() {
    m_launcher1.restoreFactoryDefaults();
    m_launcher2.restoreFactoryDefaults();
    m_launcher1.setIdleMode(IdleMode.kCoast);
    m_launcher2.setIdleMode(IdleMode.kCoast);
    m_launcher2.follow(m_launcher1, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void launch() {
    m_launcher1.set(LauncherConstants.launcherMotorSpeed);
  }

  public void unclogLauncher() {
    m_launcher1.set(- LauncherConstants.launcherMotorSpeed / 2);
  }

  public void stopLauncher() {
    m_launcher1.set(0.0);
  }
}
