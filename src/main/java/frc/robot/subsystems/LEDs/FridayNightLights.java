// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FridayNightLights implements LedsIO {

  private static final int length = 30;
  private static final int centerLED = 15;
  private static final double strobeDuration = 0.2;
  private static final double rainbowFullLength = 30.0;
  private static final double rainbowDuration = 0.25;
  private static final double breathDuration = 2.0;

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  /** Creates a new FridayNightLights. */
  public FridayNightLights() {
    leds = new AddressableLED(0); //subject to change
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
  }

    
  
  @Override
  public void setMode(LedMode mode) {
    switch(mode){
      
    }
    // This method will be called once per scheduler run
  }
}
