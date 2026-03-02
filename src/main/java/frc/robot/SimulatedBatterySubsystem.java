// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedBatterySubsystem extends SubsystemBase {
  double beginningVoltage;
  double vWithNoLoad;
  PDPSim pdpSim;
  DoubleSupplier powerValueSupplier;

  /** Creates a new SimulatedBattery. */
  public SimulatedBatterySubsystem(PowerDistribution powerDistribution, DoubleSupplier powerValueSupplier) {
    beginningVoltage = vWithNoLoad = RoboRioSim.getVInVoltage();
    if (powerDistribution != null) {
      pdpSim = new PDPSim(powerDistribution);
    }
    this.powerValueSupplier = powerValueSupplier;

    SmartDashboard.putData(Commands.runOnce(() -> { vWithNoLoad = beginningVoltage; }).ignoringDisable(true).withName("Reset Simulated Battery"));
  }

  @Override
  public void periodic() {
    double power = powerValueSupplier.getAsDouble();
    var vWithLoad = vWithNoLoad;
    if (power > 0) {
      vWithNoLoad = vWithNoLoad - 0.00125;
      vWithLoad = vWithNoLoad - 0.5;
    }

    RoboRioSim.setVInVoltage(vWithLoad);
    if (pdpSim != null) {
      pdpSim.setVoltage(vWithLoad);
      for (int i = 1; i <= 4; i++) {
        pdpSim.setCurrent(i, power * vWithLoad * 2); // make up a number
      }
    }
  }
}
