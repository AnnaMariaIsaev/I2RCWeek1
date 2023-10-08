// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class EncoderDrive extends CommandBase {

  DriveTrain dt;
  double setpoint;
  //do we need to initialize the setpoint if it's created in robot container??

  /** Creates a new encoderDrive. */
  public EncoderDrive(DriveTrain dt, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.setpoint = setpoint;
    dt.addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    dt.tankDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // diameter of the wheel * pi / ticks
    dt.tankDrive(0.3, 0.3);
    SmartDashboard.putNumber("meters", dt.ticksToMeters());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dt.ticksToMeters() >= setPoint;
  }
}
