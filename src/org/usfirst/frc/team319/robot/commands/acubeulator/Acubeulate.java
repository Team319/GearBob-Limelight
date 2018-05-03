package org.usfirst.frc.team319.robot.commands.acubeulator;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Acubeulate extends Command {

	public Acubeulate() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.acubeulator);

	}

	// Called just before this Command runs the first time
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double x = Robot.drivetrain.returnX();

		double collectValue = Robot.acubeulator.collect();
		
		

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
