// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.BNASubystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScorePose;

public class IntakeAngle extends SubsystemBase {
  // estou estabelecendo a variável para meus motores
  TalonFX m_leftAnglemotor, m_rightAnglemotor;
 // estou criando a variável para meu encoder
  DutyCycleEncoder m_Encoder;
  //Estou criando minha variavel que vai ter as contantes do pid atribuídas
  PIDController pid;


  public IntakeAngle() {
// estou declarando os ids dos meus motores
  m_leftAnglemotor = new TalonFX(12);
  m_rightAnglemotor = new TalonFX(19);

  // estou declarando respectivamente, a porta de entrada da roborio,o maximo de graus que meu encoder mede, e definir aonde é o meu ponto zero
  m_Encoder = new DutyCycleEncoder(8, 360.0,0);

  pid = new PIDController(0, 0, 0);
    
  pid.setSetpoint(ScorePose.INITAL.angle);

 
  }

  public void setSpeed(double speed){
    m_leftAnglemotor.setSpeed(speed);
    m_rightAnglemotor.setSpeed(-speed);
  }

  @Override
  public void periodic() {
      // Calcula a saída do PID com base na medição atual 
      double output = pid.calculate(getMesuarament());

          // Limita a saída do PID de -0.1 a 0.1 para evitar movimentos bruscos(quantidade de força que vou mandar para o motor agora)

      output = MathUtil.clamp(output, -0.1, 0.1 );

         // Aplica a saída do PID aos motores ( colocar o motor que esta com o encoder que vai espelhar ao outro)
        m_leftAnglemotor.set(output);
       
        // inverter valor caso necessário
        m_rightAnglemotor.set(output); // esse motor vai se espelhar o omovimento do outro motor ja que estou utilizando o encode e eles estao simetricamnete posicionado

        SmartDashboard.putNumber("Valor Encoder Intake", getMesuarament());
  }

  // cria o comando que vai ser acionado quando eu apertar um botao vinculado ao um setpoint
  public Command setSetpoint(ScorePose pose){
    return runOnce(() -> pid.setSetpoint(pose.angle));
  } 
 // Retorna o ângulo atual do pivot em graus (-180 a 180), no caso você pegaria o valor do encoder do kraken
  // VERIFICAR NECESSIDADE DISSO E SINAL NEGATIVO
  private double getMesuarament(){
    double rawAngle = m_Encoder.get();
    return - ((rawAngle > 180) ? rawAngle - 360 : rawAngle);
  }

 }
