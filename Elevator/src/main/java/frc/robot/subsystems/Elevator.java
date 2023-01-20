// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivebase extends SubsystemBase {

  //Problems: how do i transfer the data of how past smth spins to the height.
  //How to calculate what the height is.

  /** Creates a new ExampleSubsystem. */
  public Elevator() {
    motor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    //Set hax soft limit to where the limit switch is? DO i need? And whouldn't i need to somehow set that equal to the limitSwitch
    motor.setSoftLimit(SoftLimitDirection.kReverse, 1000);
  }

  //Neo motor
  CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);

  //Throughrobore encode (plugs into the front) making it a relative encoder 
  RelativeEncoder revThroughBoreEncoder = motor.getEncoder(type.kQuadrature, 8192);

  SparkMaxLimitSwitch limitSwtich = revThroughBoreEncoder.getReverseLimitSwitch(Type.kNormallyClosed);
  //isPressed();

  //PID 
  PIDController m_PIDController = new PIDController(1, 0.1, 0.1);

  public static double getHeight(){
    return revThroughBoreEncoder.getPosition();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase setElevatorHeight(double desiredHeight) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          motor.set(m_PIDController.calculate(getHeight(), desiredHeight));
        });
  }  
  //  public CommandBase elevatorMax() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         motor.set(m_PIDController.calculate(revThroughBoreEncoder., 0));
  //         //or motor.setVoltage(0);
  //       });
  // }
  // public CommandBase elevatorMid() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         motor.set(m_PIDController.calculate(double measurement, 500));
  //       });
  // }
  // public CommandBase elevatorMin() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         motor.set(m_PIDController.calculate(double measurement, 1000));
  //       });
  // }

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
    SmartDashboard.putNumber("Height", getHeight());
    Smartdashboard.putData("Max height reached", limitSwitch.isPressed());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command forward(double d) {
    return null;
  }
}
