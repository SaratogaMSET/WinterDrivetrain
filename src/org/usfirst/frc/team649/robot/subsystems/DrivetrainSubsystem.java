package org.usfirst.frc.team649.robot.subsystems;


import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivetrainSubsystem extends Subsystem {
    
    public Victor [] motors;
    public Encoder [] encoders;
    public PIDController encoderDriveLeftPID;
    
    public static double DIST_PER_PULSE_LEFT = 0.78 * 7.0/30.0 / 128.0;
    public static double DIST_PER_PULSE_RIGHT = 0.78 * 7.0/30.0 / 128.0;
    
    public static final double AUTO_P = 0.025;
	public static final double AUTO_I = 0.0015;
	public static final double AUTO_D = 0.0;
    
    
    public DrivetrainSubsystem() {
    	motors = new Victor[4];
       	motors[0] = new Victor(2);
       	motors[1] = new Victor(3);
       	motors[2] = new Victor(0);
       	motors[3] = new Victor(9);
       	
//    	encoderDriveLeftPID = this.getPIDController();
//    	encoderDriveLeftPID.setAbsoluteTolerance(0.8);
    	//encoderDrivePID.setOutputRange(-EncoderBasedDriving.MAX_MOTOR_POWER, EncoderBasedDriving.MAX_MOTOR_POWER);
    	encoders = new Encoder[2];
        encoders[0] = new Encoder(6,7, true);
        encoders[1] = new Encoder(8,9, false);
        
        
        encoders[0].setDistancePerPulse(DIST_PER_PULSE_LEFT);
        encoders[1].setDistancePerPulse(DIST_PER_PULSE_RIGHT);
        
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    

    public void driveFwdRot(double fwd, double rot) {
        double left = fwd + rot, right = fwd - rot;
        double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
        left /= max;
        right /= max;
        rawDrive(left, right);
    }

    public void rawDrive(double left, double right) {
        int i = 0;
        left /= 1.0;//left > 0 ? Math.pow(left, 2)/2 : -Math.pow(left, 2)/2;
        right /= 1.0;//right > 0 ? Math.pow(right, 2)/2 : -Math.pow(right, 2)/2;
        for (; i < motors.length / 2; i++) {
            motors[i].set(left);
        }

        for (; i < motors.length; i++) {
            motors[i].set(-right);
        }
    }
    
    public double getDistanceDTLeft() {
//        int numEncoders = encoders.length;
//        double totalVal = 0;
//        for (int i = 0; i < numEncoders; i++) {
//            totalVal += encoders[i].getDistance();
//        }
//        return totalVal / numEncoders;
        
        return encoders[0].getDistance();
        //return encoders[1].getDistance();
    }

    public double getDistanceDTRight() {
    	
      return encoders[1].getDistance();
  }
    
    public void resetEncoders() {
        encoders[0].reset();
        encoders[1].reset();
    }
    
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
//    public PIDController getGyroPIDControler() {
//    	return encoderTurnPID;
//    }
//	@Override
//	public void pidWrite(double output) {
//        
//        driveFwdRot(0, output);
//	}
//	@Override
//	public double pidGet() {
//		return this.encoders[0].getDistance();
//	}
}

