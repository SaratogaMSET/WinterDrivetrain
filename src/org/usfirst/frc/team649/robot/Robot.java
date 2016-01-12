
package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team649.robot.commands.Drive;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	 /**
	  * 
    * This function is run when the robot is first started up and should be
    * used for any initialization code.
    */
	DoubleSolenoid left, right; //grabberPiston2;
	Joystick driverRightJoystick;
	Joystick driverLeftJoystick;
	Joystick manualJoystick;
	boolean shift;
	//
	double p_value;
	double i_value;
	double d_value;
	
	public static DrivetrainSubsystem drivetrain;
	
	private boolean prevState1Button;
	private boolean prevState2Button;
	private boolean prevState5Button;
	private boolean prevState6Button;
	private boolean prevState4Button;
	
   public void robotInit() {
	drivetrain = new DrivetrainSubsystem();
	
   	driverLeftJoystick = new Joystick(0);
   	driverRightJoystick = new Joystick(1);
   	manualJoystick = new Joystick(3);
   	
   	p_value = drivetrain.getPIDController().getP();
   	i_value = drivetrain.getPIDController().getI();
   	d_value = drivetrain.getPIDController().getD();
   	
   	prevState1Button = false;
    prevState2Button = false;
    prevState5Button = false;
    prevState6Button = false;
    prevState4Button = false;
   	
   	left = new DoubleSolenoid(6, 7);
   	right = new DoubleSolenoid(4,5);
   }
   
   
   /**
    * This function is called periodically during autonomous
    */
   public void autonomousPeriodic() {

   }

   /**
    * This function is called periodically during operator control
    */
   public void teleopPeriodic() {
   	shift = driverRightJoystick.getRawButton(1) || driverLeftJoystick.getRawButton(1);
   	drivetrain.driveFwdRot(getDriveForward(), getDriveRotation());
   	shiftDriveGear(shift);
   	Command d;
   	
   	if (manualJoystick.getRawButton(1) && !prevState1Button){
    	d = new Drive(10);
    	d.start();
    }
    else if (manualJoystick.getRawButton(2) && !prevState2Button){
    	d = new Drive(-10);
    	d.start();
    }
    
    //set PID
    //on GAMEPAD: P:LB, I:RB, D:Y-button
    //			DECREASE^^ by simultaneously holding X button
    if(manualJoystick.getRawButton(5) && !prevState5Button){
    	p_value+= (manualJoystick.getRawButton(3)? -0.1 : 0.1);
    }
    if(manualJoystick.getRawButton(6) && !prevState6Button){
    	i_value+= (manualJoystick.getRawButton(3)? -0.1 : 0.1);
    }
    if(manualJoystick.getRawButton(4) && !prevState4Button){
    	d_value+= (manualJoystick.getRawButton(3)? -0.1 : 0.1);
    }
    
    //check to make sure none are under 0
    if (p_value < 0) p_value=0;
    if (i_value < 0) i_value=0;
    if (d_value < 0) d_value=0;
    
    //update PID
    drivetrain.getPIDController().setPID(p_value, i_value, d_value);
    
    prevState1Button = manualJoystick.getRawButton(1);
    prevState2Button = manualJoystick.getRawButton(2);
    prevState5Button = manualJoystick.getRawButton(5);
    prevState6Button = manualJoystick.getRawButton(6);
    prevState4Button = manualJoystick.getRawButton(4);
   	
    SmartDashboard.putNumber("P", p_value); 
    SmartDashboard.putNumber("I", i_value);
    SmartDashboard.putNumber("D", d_value);
   	SmartDashboard.putData("Encoder Right", drivetrain.encoders[0]);
   }
   
   /**
    * This function is called periodically during test mode
    */
   public void testPeriodic() {
   
   }
   
   public double getDriveForward() {
       return -driverLeftJoystick.getY();
   }

   public double getDriveRotation() {
       final double turnVal = driverRightJoystick.getX();
       final double sign = turnVal < 0 ? -1 : 1;
       return Math.pow(Math.abs(turnVal), 1.4) * sign;
   }
   
   public void shiftDriveGear(boolean lowSpeed) {
   	//default low
       left.set(lowSpeed ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
       right.set(lowSpeed ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);

   }
}