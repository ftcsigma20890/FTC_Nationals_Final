//package org.firstinspires.ftc.teamcode.Mechanisms;
//
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//public class ColorCommand {
//
//   private NormalizedColorSensor colorSensor;
//   private DigitalChannel led;
//   public boolean ledState;
//
//
//   public ColorCommand(HardwareMap hardwareMap, String sensorName) {
//      colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorName);
//      colorSensor.setGain(2.0f);
//   }
//
//   private NormalizedRGBA getColors() {
//      return colorSensor.getNormalizedColors();
//   }
//
//   private float red() {
//      return getColors().red * 255;
//   }
//
//   private float green() {
//      return getColors().green * 255;
//   }
//
//   private float blue() {
//      return getColors().blue * 255;
//   }
//
//   public boolean isGreen() {
//      float r = red();
//      float g = green();
//      float b = blue();
//
//      return (g > 120 && g > r * 1.3 && g > b * 1.3);
//   }
//
//   public boolean isPurple() {
//      float r = red();
//      float g = green();
//      float b = blue();
//
//      return (r > 100 && b > 100 && Math.abs(r - b) < 40 && g < 80);
//   }
//
//   public void updateLED() {
//      if (isGreen() == true || isPurple() == true){
//         led.setState(true);
//      }else{
//         led.setState(false);
//      }
//   }
//   public void checkledstate(){
//      led.getState(ledState);
//   }
//}
