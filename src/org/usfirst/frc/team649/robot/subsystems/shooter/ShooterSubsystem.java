package org.usfirst.frc.team649.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
	public static double AUTO_P;
	public static double AUTO_I;
	public static double AUTO_D;
	
	public static double TARGET_SHOOT_SPEED = 1900; //rpm : 4500 max, 1900 worked
	
	public Victor [] rollers;
	
	public Victor leverArmPivot;
	public AnalogInput absEncoder;
	public DoubleSolenoid shootSol;
	
	public Counter photoSensor1;
	public Counter photoSensor2;
	
	public ShooterSubsystem(){
		rollers = new Victor[2];
		rollers[0] = new Victor(1);   
		rollers[1] = new Victor(4);
		
		photoSensor1 = new Counter(1);
		photoSensor2 = new Counter(0);
		
		shootSol = new DoubleSolenoid(0,1);
		
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

	public double getRPMEinstein1() {
		// TODO Auto-generated method stub
		return 60.0/photoSensor1.getPeriod();
	}
	public double getRPMEinstein2() {
		// TODO Auto-generated method stub
		return 60.0/photoSensor2.getPeriod();
	}
}
