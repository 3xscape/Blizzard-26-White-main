// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.jar.Attributes.Name;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.LimelightHelpers;

/** Add your docs here. */
public class Limelight {
    
    public enum LimelightType{
        LL4("limelight-new", 0);

        private final String LLname;
        private final double deadband;

        LimelightType(String LLname, double deadband){
            this.LLname = LLname;
            this.deadband = deadband;
        }

        public String getName(){
        return LLname;
      }

        public double getDeadband(){
            return deadband;
        }
    }
    private final String name;
    private final double deadband; 

    private final ProfiledPIDController turnController;
    private final ProfiledPIDController idleturnController;

    public static Limelight createLL4(){
        LimelightType config = LimelightType.LL4;
        return new Limelight(
            config.getName(),
            config.getDeadband(),
            0.1,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(2,2), 
            0.1,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(2,2)
        );
    }

        
    
    private Limelight(
        String name,
        double deadband,
        double leftRightKp,
        double leftRightKi,
        double leftRightKd,
        TrapezoidProfile.Constraints leftRightConstraints,
        double idleLeftRighttKp,
        double idleLeftRighttKi,
        double idleLeftRighttKd,
        TrapezoidProfile.Constraints idleLeftRighttConstraints){
            this.name = name;
            this.deadband = deadband;

            this.turnController = new ProfiledPIDController(leftRightKp, leftRightKi, leftRightKd, leftRightConstraints);
            this.idleturnController = new ProfiledPIDController(idleLeftRighttKp, idleLeftRighttKi, idleLeftRighttKd, idleLeftRighttConstraints);
            
        }

    public int getId() {
    return (int) LimelightHelpers.getFiducialID(this.name);
  }
  
    public double getAimError() {
        if (getId() == -1 || getId() == 0) {
        return 0;
        }
        double error = LimelightHelpers.getTX(name);
        return MathUtil.isNear(0, error, deadband) ? 0 : error;
    }
  
    public double calculateLeftRightOutput() {
    return turnController.calculate(getAimError());
  }

  public double calculateIdleLeftRightOutput() {
    return idleturnController.calculate(getAimError());
  }
}