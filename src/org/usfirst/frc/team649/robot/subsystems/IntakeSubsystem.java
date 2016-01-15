package org.usfirst.frc.team649.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

public class IntakeSubsystem extends Subsystem {
	
	public static double INTAKE_SPEED = 1.0;
	public static double PURGE_SPEED = -1.0;
	Victor [] rollers;
	
	public IntakeSubsystem(){
		rollers = new Victor[3];
		rollers[0] = new Victor(7); //one of horizontal spins
		rollers[1] = new Victor(8); //other horizontal
		rollers[2] = new Victor(10); //vertical spin
	}
	
	public void setRollerSpeed(double speed){
		
		rollers[0].set(speed);
		rollers[1].set(-speed);
		rollers[2].set(speed);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
