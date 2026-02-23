// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc3620.Utilities.SlidingWindowStats;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeaterSubsystem extends SubsystemBase {
  /** Creates a new HeaterSubsystem. */
  List<WPI_TalonSRX> heaters = new ArrayList<>();
  PowerDistribution powerDistribution;
  int hb = 0;

  SlidingWindowStats v25 = new SlidingWindowStats(25);
  SlidingWindowStats v50 = new SlidingWindowStats(50);
  SlidingWindowStats v100 = new SlidingWindowStats(100);

  public HeaterSubsystem() {
    powerDistribution = RobotContainer.powerDistribution;
    heaters.add(new WPI_TalonSRX(1));
    heaters.add(new WPI_TalonSRX(2));
    heaters.add(new WPI_TalonSRX(3));
    heaters.add(new WPI_TalonSRX(4));
  }

  void setHeaters(double value) {
    for (var heater : heaters) {
      heater.set(TalonSRXControlMode.PercentOutput, value);
    }
  }

  public Command makeSetSpeedCommand(double value) {
    return run(() -> setHeaters(value));
  }

  @Override
  public void periodic() {
    hb = hb + 1;
    double[] currents = null;
    if (powerDistribution != null) {
      currents = powerDistribution.getAllCurrents();
    }

    for (var heater : heaters) {
      int deviceId = heater.getDeviceID();
      String name = "H" + deviceId;

      double outputCurrent = heater.getStatorCurrent();
      double outputVoltage = heater.getMotorOutputVoltage();

      DogLog.log(name + "/output/v", outputVoltage);
      DogLog.log(name + "/output/a", outputCurrent);
      DogLog.log(name + "/output/w", outputCurrent*outputVoltage);
      DogLog.log(name + "/output/hb", hb);
      
      double inputCurrent = heater.getSupplyCurrent();
      double inputVoltage = heater.getBusVoltage();

      DogLog.log(name + "/input/v", inputVoltage);
      DogLog.log(name + "/input/a", inputCurrent);
      DogLog.log(name + "/input/w", inputCurrent*inputVoltage);
      DogLog.log(name + "/input/hb", hb);

      double setpoint = heater.getMotorOutputPercent();
      DogLog.log(name + "/setpoint", setpoint);

      // heater 1 is wired to PDP port 0, heater 2 to PDP port 1, etc.
      if (currents != null) {
      DogLog.log(name + "/pdb/a", currents[deviceId-1]);
      }
    }

    if (powerDistribution != null) {
      double v = powerDistribution.getVoltage();
      double a = powerDistribution.getTotalCurrent();
      DogLog.log("pdb/v", v);
      DogLog.log("pdb/a", a);
      DogLog.log("pdb/w", v*a);
      DogLog.log("pdb/w_pdb", powerDistribution.getTotalPower());
      DogLog.log("pdb/j", powerDistribution.getTotalEnergy());
      DogLog.log("pdb/hb", hb);
    }
    double v = RobotController.getBatteryVoltage();
    DogLog.log("roborio/v", v);
    v25.addValue(v);
    DogLog.log("roborio/v25", v25.getMean());
    v50.addValue(v);
    DogLog.log("roborio/v50", v50.getMean());
    v100.addValue(v);
    DogLog.log("roborio/v100", v100.getMean());
    DogLog.log("hb", hb);

    Command c = this.getCurrentCommand();
    DogLog.log("cmd", c == null ? "" : c.getName());
  }

  public double getBatteryVoltage() {
    return v100.getMean();
  }
}
