// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it undere the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Drivebase() {
    m_rightMaster.setInverted(true);
  }

  CANSparkMax m_leftMotor1 = new CANSparkMax(0, MotorType.kBrushed);
  CANSparkMax m_leftMotor2 = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax m_leftMotor3 = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax m_rightMotor1 = new CANSparkMax(3, MotorType.kBrushed);
  CANSparkMax m_rightMotor2 = new CANSparkMax(4, MotorType.kBrushed);
  CANSparkMax m_rightMotor3 = new CANSparkMax(5, MotorType.kBrushed);

  MotorControllerGroup m_leftMaster = new MotorControllerGroup(m_leftMotor1, m_leftMotor2, m_leftMotor3);
  MotorControllerGroup m_rightMaster = new MotorControllerGroup(m_rightMotor1, m_rightMotor2, m_rightMotor3);

  DifferentialDrive diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public CommandBase forward(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          diffDrive.arcadeDrive(speed, 0);
        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
