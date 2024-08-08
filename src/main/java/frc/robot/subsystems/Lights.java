// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class Lights extends SubsystemBase {

  PneumaticsSystem pneumatics;

  // "slope" is the ratio of the PSI to the Hue.
  // ps. Couldn't be bothered to understand the math, but I used this
  // https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
  private float slope = (float) ((80.0 - 10) / (70.0 - 0));
  private float inputPSI;
  private int outputHue;

  public enum LEDState {
    rainbow,
    pinkSolid,
    pinkWhiteGradient,
    pressureRedtoGreenGradient
  }

  AddressableLED leds = new AddressableLED(0);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(175);
  int rainbowFirstPixelHue = 1;
  int pinkWhiteGradientFirstPixelSaturation = 1;
  int pressureRedtoGreenGradientFirstPixelSaturation = 1;

  /** Creates a new Lights. */
  public Lights(PneumaticsSystem pneumatics) {

    this.pneumatics = pneumatics;

    leds.setLength(ledBuffer.getLength());
    leds.start();
  }

  public void lightsDefaultMethod() {
    this.changeLEDState(LEDState.rainbow);
  }

  public void changeLEDState(LEDState state) {

    switch (state) {
      case rainbow:
        WrapperCommand rainbowCommand =
            new RunCommand(
                    () -> {
                      // For every pixel
                      for (var i = 0; i < ledBuffer.getLength(); i++) {
                        // Calculate the hue - hue is easier for rainbows because the color
                        // shape is a circle so only one value needs to precess
                        final var hue1 =
                            (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
                        // Set the value
                        ledBuffer.setHSV(i, hue1, 255, 128);
                      }
                      // Increase by to make the rainbow "move"
                      rainbowFirstPixelHue += 1;
                      // System.out.println(rainbowFirstPixelHue);
                      // Check bounds
                      rainbowFirstPixelHue %= 180;
                      leds.setData(ledBuffer);
                    },
                    this)
                .withName("Rainbow Lights");

        rainbowCommand.schedule();
        System.out.println("rainbow!");

        break;

      case pinkSolid:
        WrapperCommand pinkSolidCommand =
            new RunCommand(
                    () -> {
                      for (var i = 0; i < ledBuffer.getLength(); i++) {
                        ledBuffer.setLED(i, Color.kLightPink);
                      }
                    },
                    this)
                .withName("Solid Pink Lights");

        pinkSolidCommand.schedule();
        leds.setData(ledBuffer);

        break;

      case pinkWhiteGradient:
        WrapperCommand pinkWhiteGradientCommand =
            new RunCommand(
                    () -> {
                      // For every pixel
                      for (var i = 0; i < ledBuffer.getLength(); i++) {
                        // Calculate the hue - hue is easier for rainbows because the color
                        // shape is a circle so only one value needs to precess
                        final var saturation1 =
                            (pinkWhiteGradientFirstPixelSaturation
                                    + (i * 210 / ledBuffer.getLength()))
                                % 210;
                        // Set the value
                        ledBuffer.setHSV(i, outputHue, 225, saturation1);
                      }
                      // Increase by to make the rainbow "move"
                      pinkWhiteGradientFirstPixelSaturation += 1;
                      // System.out.println(rainbowFirstPixelHue);
                      // Check bounds
                      pinkWhiteGradientFirstPixelSaturation %= 210;

                      leds.setData(ledBuffer);
                    },
                    this)
                .withName("Pink White Gradient Lights");

        pinkWhiteGradientCommand.schedule();
        System.out.println("Pink White Gradient!");

        break;

      case pressureRedtoGreenGradient:
        WrapperCommand pressureRedtoGreenGradientCommand =
            new RunCommand(
                    () -> {
                      // For every pixel
                      for (var i = 0; i < ledBuffer.getLength(); i++) {
                        // Calculate the hue - hue is easier for rainbows because the color
                        // shape is a circle so only one value needs to precess
                        final var saturation2 =
                            (pressureRedtoGreenGradientFirstPixelSaturation
                                    + (i * 128 / ledBuffer.getLength()))
                                % 128;
                        // Set the value

                        int testS = 255;
                        int testV = 255;

                        ledBuffer.setHSV(i, outputHue, testS, testV);
                      }
                      // Increase by to make the rainbow "move"
                      pressureRedtoGreenGradientFirstPixelSaturation += 1;
                      // System.out.println(rainbowFirstPixelHue);
                      // Check bounds
                      pressureRedtoGreenGradientFirstPixelSaturation %= 128;

                      leds.setData(ledBuffer);
                    },
                    this)
                .withName("Pressure Red to Green Gradient Lights");

        pressureRedtoGreenGradientCommand.schedule();
        System.out.println("Pressure Red to Green Gradient!");

        break;
    }
  }

  private void pinkPulse() {

    int randomLED = (int) (Math.random() * ledBuffer.getLength());

    int r = 255;
    int g = 220;
    int b = 255;
    while (g > 0) {}
  }

  private void spread(int mainPoint) {

    for (int i = 0; i < 5; i++) {}
  }

  private void convertHSV() {
    inputPSI = pneumatics.psi;
    SmartDashboard.putNumber("Pressure Proportional Input", inputPSI);
    outputHue = Math.round(((0 + slope) * (inputPSI - 3)));
    SmartDashboard.putNumber("Pressure Proportional Hue", outputHue);
  }

  @Override
  public void periodic() {
    convertHSV();
    // System.out.println(slope);
    // System.out.println("test");
    // This method will be called once per scheduler run

  }
}
