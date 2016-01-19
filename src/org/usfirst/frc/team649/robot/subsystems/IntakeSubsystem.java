package org.usfirst.frc.team649.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

public class IntakeSubsystem extends Subsystem {
	
	public static double INTAKE_SPEED = 1.0;
	public static double PURGE_SPEED = -1.0;
	Victor [] rollers;
	
	public IntakeSubsystem(){
		//	 FOR TESTING OF INTAKE SYSTEM
		rollers = new Victor[3];
		rollers[0] = new Victor(15); //horizontal spin 7
		rollers[1] = new Victor(10); //horizontal spin
		rollers[2] = new Victor(16); //vertical spin 8
	}
	
	public void setRollerSpeed(double speed){
		
		rollers[0].set(-speed*.7);
		rollers[1].set(speed*.7);
		rollers[2].set(speed);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
