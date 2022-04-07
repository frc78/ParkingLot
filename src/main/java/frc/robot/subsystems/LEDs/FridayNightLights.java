// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FridayNightLights implements LedsIO {

  private static final int length = 30;
  private static final int centerLED = 15;
  private static final double strobeDuration = 0.2;
  private static final double rainbowFullLength = 30.0;
  private static final double rainbowDuration = 0.25;
  private static final double breathDuration = 2.0;
  private static final double waveExponent = 0.4;
  private static final int halfLength = (int) Math.ceil(length / 2.0);

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
      case Shoot_Accuracy:
      breath(Color.kGreen, Color.kBlue);
      break;

    }
    // This method will be called once per scheduler run
  }
  private void wave(Color c1, Color c2, double fullLength, double duration){
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / fullLength;
    for (int i = 0; i < halfLength; i++){
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if(Double.isNaN(ratio)){
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)){
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setLedsSymmetrical(i, new Color(red, green, blue));
    }
  } 
  private void setLedsSymmetrical(int index, Color color){
    buffer.setLED((centerLED + index) % length, color);
    buffer.setLED(centerLED - index, color);
  }
  public void solid(Color color){
    for (int i = 0; i < length; i ++) {
      buffer.setLED(1, color);
    }
  }
  private void breath(Color c1, Color c2){
    double x = ((Timer.getFPGATimestamp() % breathDuration)/ breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x)) + 1.0 / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.blue * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
    
  }
}
