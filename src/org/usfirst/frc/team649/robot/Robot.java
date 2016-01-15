
package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team649.robot.commands.DrivePIDLeft;
import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.DrivePIDRight;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LeftDTPID;
import org.usfirst.frc.team649.robot.subsystems.RightDTPID;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public static double DEAD_ZONE_TOLERANCE = 0.025;
	
	 /**
	  * 
    * This function is run when the robot is first started up and should be
    * used for any initialization code.
    */
	DoubleSolenoid left, right; //grabberPiston2;
	Joystick driverRightJoystick;
	Joystick driverLeftJoystick;
	Joystick manualJoystick; 
	Joystick operatorJoystick;
	boolean shift;
	//
	double p_value;
	double i_value;
	double d_value;
	
	public static DrivetrainSubsystem drivetrain;
	public static IntakeSubsystem intake;
	public static ShooterSubsystem shooter;
	public static LeftDTPID leftDT;
	public static RightDTPID rightDT;
	
	private boolean prevStateLeft6Button;
	private boolean prevStateLeft7Button;
	private boolean prevStateRight11Button;
	private boolean prevStateRight10Button;
	
	private boolean prevState3Button;
	private boolean prevState1Button;
	private boolean prevState2Button;
	
	public static double INCREMENT = 0.001;
	public static boolean isPIDActiveLeft = false;
	public static boolean isPIDActiveRight = false;
	
	
   public void robotInit() {
	drivetrain = new DrivetrainSubsystem();
	shooter = new ShooterSubsystem();
	intake = new IntakeSubsystem();
	
	leftDT = new LeftDTPID();
	rightDT = new RightDTPID();
	
   	driverRightJoystick = new Joystick(0);
   	driverLeftJoystick = new Joystick(1);
   	operatorJoystick = new Joystick(2);
   	manualJoystick = new Joystick(3);
   	
   	p_value = leftDT.getPIDController().getP();
   	i_value = leftDT.getPIDController().getI();
   	d_value = leftDT.getPIDController().getD();
   	
   	prevStateLeft6Button = false;
    prevStateLeft7Button = false;
    prevStateRight11Button = false;
    prevStateRight10Button = false;
    prevState3Button = false;
    prevState1Button = false;
    prevState2Button = false;
   	
   	left = new DoubleSolenoid(6, 7);
   	right = new DoubleSolenoid(4,5);
   }
   
   public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}
   
   /**
    * This function is called periodically during autonomous
    */
   public void autonomousPeriodic() {
	   Scheduler.getInstance().run();
	   
   }

   public void teleopInit(){
	   drivetrain.resetEncoders();
   }
   /**
    * This function is called periodically during operator control
    */
   public void teleopPeriodic() {
	   Scheduler.getInstance().run();
	   
	   shift = driverRightJoystick.getRawButton(1) || driverLeftJoystick.getRawButton(1);
	   if (!(isPIDActiveLeft || isPIDActiveRight) ){
		   new DriveForwardRotate(getDriveForward(), getDriveRotation()).start();
	   }
	   shiftDriveGear(shift);
	   
	   //shooter
	   if (operatorJoystick.getRawButton(1)){
		   shooter.setRollerSpeed(correctForDeadZone( (-operatorJoystick.getThrottle() + 1.0) / 2.0));
	   }
	   
	   //intakes
	   if (operatorJoystick.getRawButton(11)){
		   intake.setRollerSpeed(IntakeSubsystem.INTAKE_SPEED);
	   }
	   else if (operatorJoystick.getRawButton(12)){
		   intake.setRollerSpeed(IntakeSubsystem.PURGE_SPEED);
	   }
	   else{
		   intake.setRollerSpeed(0);
	   }
	   
	   SmartDashboard.putNumber("Throttle", -operatorJoystick.getThrottle());
	   SmartDashboard.putBoolean("Trigger", operatorJoystick.getRawButton(1));
	   
	   //auto pid tuning
	   Command d_l, d_l_2, d_r, d_r_2;
	   
	   /*
	    *		OLD MANUAL JOYSTICK SET UP FOR DRIVE PID
	    *
	   if (manualJoystick.getRawButton(1) && !prevState1Button){
		   d_l = new DrivePIDLeft(20);
		   d_l.start();
	   }
	   else if (manualJoystick.getRawButton(2) && !prevState2Button){
		   d_l = new DrivePIDLeft(-20);
		   d_l.start();
	   }
	    */
	   
	   
	   //left side (
	   if (driverLeftJoystick.getRawButton(6) && !prevStateLeft6Button){
		   d_l = new DrivePIDLeft(20);
		   d_l.start();
	   }
	   else if (driverLeftJoystick.getRawButton(7) && !prevStateLeft7Button){
		   d_l_2 = new DrivePIDLeft(-20);
		   d_l_2.start();
	   }
	   
	   //right side
	   if (driverRightJoystick.getRawButton(11) && !prevStateRight11Button){
		   d_r = new DrivePIDRight(20);
		   d_r.start();
	   }
	   else if (driverRightJoystick.getRawButton(10) && !prevStateRight10Button){
		   d_r_2 = new DrivePIDRight(-20);
		   d_r_2.start();
	   }
	   
	   //set PID
	   //on GAMEPAD: P:X-button, I:A-button, D:Y-button
	   //			DECREASE^^ by simultaneously holding LB button
	   if(manualJoystick.getRawButton(3) && !prevState3Button){
		   p_value+= (manualJoystick.getRawButton(5)? -INCREMENT : INCREMENT);
	   }
	   if(manualJoystick.getRawButton(1) && !prevState1Button){
		   i_value+= (manualJoystick.getRawButton(5)? -INCREMENT : INCREMENT);
	   }
	   if(manualJoystick.getRawButton(2) && !prevState2Button){
		   d_value+= (manualJoystick.getRawButton(5)? -0.0001 : 0.0001);
	   }
		
	   //check to make sure none are under 0
	   if (p_value < 0) p_value=0;
	   if (i_value < 0) i_value=0;
	   if (d_value < 0) d_value=0;
	   
	   //update PID
	   leftDT.getPIDController().setPID(p_value, i_value, d_value);
	   rightDT.getPIDController().setPID(p_value, i_value, d_value);
	   
	   //for drive pid commands
	   prevStateLeft6Button = driverLeftJoystick.getRawButton(1);
	   prevStateLeft7Button = driverLeftJoystick.getRawButton(2);
	   prevStateRight11Button = driverRightJoystick.getRawButton(11);
	   prevStateRight10Button = driverRightJoystick.getRawButton(10);
	   //PID tuning
	   prevState3Button = manualJoystick.getRawButton(3);
	   prevState1Button = manualJoystick.getRawButton(1);
	   prevState2Button = manualJoystick.getRawButton(2);
	   
	   SmartDashboard.putNumber("P", p_value); 
	   SmartDashboard.putNumber("I", i_value);
	   SmartDashboard.putNumber("D", d_value);
	   SmartDashboard.putData("Encoder Left", drivetrain.encoders[0]);
	   SmartDashboard.putData("Encoder Right", drivetrain.encoders[1]);
	   SmartDashboard.putBoolean("Is PID Active?", isPIDActiveLeft);
   }
   
   /**
    * This function is called periodically during test mode
    */
   public void testPeriodic() {
	   Scheduler.getInstance().run();
	  
   }
   
   
   // *******************************ADDED FUNCTIONS******************************//
   
   public double correctForDeadZone(double joyVal, double minTolerance){
	   return Math.abs(joyVal) >= minTolerance ? joyVal : 0;
   }
   
   //default tolerance
   public double correctForDeadZone(double joyVal){
	   return Math.abs(joyVal) >= DEAD_ZONE_TOLERANCE ? joyVal : 0;
   }
   
   public double getDriveForward() {
       return correctForDeadZone(-driverLeftJoystick.getY());
   }

   public double getDriveRotation() {
       final double turnVal = correctForDeadZone(driverRightJoystick.getX());
       final double sign = turnVal < 0 ? -1 : 1;
       return Math.pow(Math.abs(turnVal), 1.4) * sign;
   }
   
   public void shiftDriveGear(boolean lowSpeed) {
   	//default low
       left.set(lowSpeed ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
       right.set(lowSpeed ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
       
       //for future implementation of shifting DT with solenoids and correcting encoders accordingly
//       if (!lowSpeed && prevSolState){
//    	   encoder.setDistancePerPulse(asfnsfna,s);
//       }

   }
}