// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase FowardFor3Secs(Drivebase m_db) {
    return Commands.sequence(
      m_db.forward(1.0).withTimeout(3), 
      m_db.forward(0.0)
      );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
