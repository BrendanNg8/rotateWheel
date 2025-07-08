// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class rotationSub extends SubsystemBase {
  /** Creates a new rotationSub. */
  TalonFX motor;
  double kp = 0.005;
  double ki = 0.0;
  double kd = 0.00000000;
  Joystick joy;
  // PID controller for the motor
  // Adjust these values based on your system's requirements
  PIDController pid; 
  

  CANcoder encoder;
  public rotationSub(Joystick j, CANcoder c) {
    motor = new TalonFX(1, "Drivetrain");
    pid = new PIDController(kp, ki, kd);
    pid.enableContinuousInput(0, 360);
    joy = j;
    encoder = c;
  }
  //Was not made by kanna
  public void setPID(double p, double i, double d) {
    // Set the PID coefficients
    kp = p;
    ki = i;
    kd = d;
    pid.setPID(kp, ki, kd);
  }
  public void setMotorSpeed(double speed) {
    // Set the motor speed
    motor.set(speed);
  }
  public double getEncoderPosition() {
    // Get the encoder position
    return ((((encoder.getPosition().getValueAsDouble()%1)+1)%1)*360);
  }
  /* rotateMotor(Joystick j, CANcoder c,) {
    // This method will rotate the motor based on joystick input
    double speed = j.getY(); // Assuming Y-axis controls the speed
    motor.set(speed);
    
    // Optionally, you can read the encoder value
    double position = c.getPosition();
    System.out.println("Encoder Position: " + position);
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngle = getEncoderPosition();
    double targetAngle = -joy.getDirectionDegrees()+180; // Example target angle
    double output = pid.calculate(currentAngle, targetAngle);
    setMotorSpeed(output);
  }
}
