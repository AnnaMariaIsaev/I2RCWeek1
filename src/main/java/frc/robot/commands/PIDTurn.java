// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.KPConstants;
import frc.robot.subsystems.DriveTrain;

public class PIDTurn extends CommandBase {
  /** Creates a new PIDTurn. */
  DriveTrain dt;
  double setpointAngle;
  int motorSign;

  /*to find the P constant motor power / error
  motor power is setpoint from 0 */

  PIDController PID = new PIDController(KPConstants.kP, 0, 0);

  public PIDTurn(DriveTrain dt, double setpointAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.setpointAngle = setpointAngle;

    addRequirements(dt);

    //setting tolerance
    PID.setTolerance(KPConstants.positionTolerance);

    //counterclockwise turn
  if(setpointAngle >= 0){
    motorSign = 1;
  } else {
    //clockwise
    motorSign = -1;
  }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavx();
    dt.tankDrive(0, 0);
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = PID.calculate(dt.getAngle(), setpointAngle);
    dt.tankDrive(-motorSign*output, motorSign*output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PID.atSetpoint();
  }
}
