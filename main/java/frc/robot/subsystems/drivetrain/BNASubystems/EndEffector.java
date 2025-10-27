// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.BNASubystems;

import java.util.Queue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class EndEffector extends SubsystemBase {
  SparkMax Neo1;
  SparkMax Neo2;
  DigitalInput InfraRed;
  DigitalInput InfraRed2;
  

   // Tempo em segundos que os motores vão rodar depois da detecção
   private static final double RUN_TIME = 1.;

  public EndEffector() {
    Neo1 = new SparkMax(2,MotorType.kBrushless);
    Neo2 = new SparkMax(3, MotorType.kBrushless);
    InfraRed = new DigitalInput(0);
    InfraRed2 = new DigitalInput(1);
    timer = new Timer();
    running = false;



  }

  // Define a velocidade dos motores do intake
  public void setSpeed(double speed) {
    Neo1.set(speed);
    Neo2.set(speed);
  }

    // Retorna se há objeto detectado
    public boolean isObjectDetected() {
      return !InfraRed.get(); // true = objeto detectado
    }

     // 🔹 Faz os motores girarem por um tempo após o IR detectar
     private static final double RUN_SPEED = 1.0; // velocidade do motor
     private static final double MIN_RUN_TIME = 1.0; // tempo mínimo do impulso
     private Timer timer = new Timer();
     private boolean running = false;
     
     public void runWithIR() {
      boolean detectedStart = !InfraRed.get();   // Sensor 1 detectou (início)
      boolean detectedEnd = !InfraRed2.get();    // Sensor 2 detectou (fim)
  
      if (detectedStart && !running) {
          // Se o primeiro sensor detectou e ainda não estava rodando
          running = true;
          timer.reset();
          timer.start();
      }
  
      if (running) {
          // Enquanto estiver rodando, gira os motores
          Neo1.set(RUN_SPEED);
          Neo2.set(RUN_SPEED);
  
          // Se o segundo sensor detectou, paramos
          if (detectedEnd) {
              Neo1.set(0);
              Neo2.set(0);
              running = false;
              timer.stop();
          }
      }
  }
         
     
  
    @Override
public void periodic() {
    SmartDashboard.putBoolean("Leitura Bruta IR", InfraRed.get());
    runWithIR(); // Agora o sensor é monitorado automaticamente
}

    }
