package org.usfirst.frc.team649.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
	public Victor [] rollers;
	
	public Victor leverArmPivot;
	public AnalogInput absEncoder;
	
	public ShooterSubsystem(){
		rollers = new Victor[2];
		rollers[0] = new Victor(15);   
		rollers[1] = new Victor(16);
		
		//leverArmPivot = new Victor(1);
		absEncoder = new AnalogInput(0);
	}
	
	public void setRollerSpeed(double speed){
		
		rollers[0].set(speed);
		rollers[1].set(-speed);
	}

	public double getAbsoluteEncoderVal(){
		return absEncoder.getValue();
	}
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
