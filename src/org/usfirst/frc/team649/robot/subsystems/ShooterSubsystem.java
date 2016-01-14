package org.usfirst.frc.team649.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
	Victor [] rollers;
	
	public ShooterSubsystem(){
		rollers = new Victor[2];
		rollers[0] = new Victor(7);
		rollers[1] = new Victor(8);
	}
	
	public void setRollerSpeed(double speed){
		
		rollers[0].set(speed);
		rollers[1].set(-speed);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
