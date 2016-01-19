package org.usfirst.frc.team649.robot.subsystems.shooter;

import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class LeftFlywheels extends PIDSubsystem {

    // Initialize your subsystem here
    public LeftFlywheels() {
    	super("Flywheels Left", ShooterSubsystem.AUTO_P, DrivetrainSubsystem.AUTO_I, DrivetrainSubsystem.AUTO_D);

        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	return 0.0;
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }
}
