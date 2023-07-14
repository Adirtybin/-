// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatic extends SubsystemBase {
  
  private Compressor m_compressor = new Compressor(0,PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid m_intakeUD = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,1);
  
  /** Creates a new Pneumatic. */
  public Pneumatic() {
    
    m_intakeUD.set(DoubleSolenoid.Value.kOff);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    
    if(!m_compressor.getPressureSwitchValue()){
      m_compressor.enableDigital();
    }else{
      m_compressor.disable();
    }
    
  
  }
  
  public void intakeUp(){
    m_intakeUD.set(DoubleSolenoid.Value.kForward);
  }

  public void intakeDown(){
    m_intakeUD.set(DoubleSolenoid.Value.kReverse);
  }
  
}
