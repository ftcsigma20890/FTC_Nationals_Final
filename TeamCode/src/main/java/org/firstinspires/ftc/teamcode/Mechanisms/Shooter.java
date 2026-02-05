package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@Configurable
public class Shooter {
   public static double kp = 250;
   public static double ki = 0;
   public static double kd = 0;
   public static double kf = 15;
   public static long shootDelay = 150;
   public static double shootRampPosition = 0.09;
   public static double shootPosition = 0.700;
   public static double longshootpos = 0;
   public static double shortshootpos = 0.3;
   public static double divideShootTime = 1;
   public static double longvelocity = 1750;
   public static double shortvelocity = 1450;
   public double velocity = 1400, hood = 0;
   DcMotorEx leftMotor, rightMotor;

   Servo hoodServo, ramp;

   public Shooter(HardwareMap hardwareMap) {
      leftMotor = hardwareMap.get(DcMotorEx.class, "leftshoot");
      rightMotor = hardwareMap.get(DcMotorEx.class, "rightshoot");
      rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      ramp = hardwareMap.get(Servo.class, "ramp");
      ramp.setPosition(0);
      hoodServo = hardwareMap.get(Servo.class, "Hood2");
      hoodServo.setPosition(0);
      leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftMotor.setPIDFCoefficients(
         DcMotor.RunMode.RUN_USING_ENCODER,
         new PIDFCoefficients(kp, ki, kd, kf)
      );
      rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightMotor.setPIDFCoefficients(
         DcMotor.RunMode.RUN_USING_ENCODER,
         new PIDFCoefficients(kp, ki, kd, kf)
      );
   }

   public void startShooter() {
      leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftMotor.setPIDFCoefficients(
         DcMotor.RunMode.RUN_USING_ENCODER,
         new PIDFCoefficients(kp, ki, kd, kf)
      );
      rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightMotor.setPIDFCoefficients(
         DcMotor.RunMode.RUN_USING_ENCODER,
         new PIDFCoefficients(kp, ki, kd, kf)
      );

      leftMotor.setVelocity(velocity);
      rightMotor.setVelocity(velocity);
      setHoodPosition(hood);
   }

   public void stopShooter() {
      leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      leftMotor.setPower(0);
      rightMotor.setPower(0);
   }

   public double getVelocity() {
      return leftMotor.getVelocity();
   }

   public double getTargetVelocity() {
      return velocity;
   }

   public double getHoodPosition() {
      return hood;
   }

   public void setHoodPosition(double position) {
      hoodServo.setPosition(position);
   }

   public boolean isVelocityReached() {
      return Math.abs(getVelocity() - velocity) < 20;
   }

   public void startLongShooter() {
      this.velocity = longvelocity;
      this.hood = longshootpos;
      startShooter();
   }

   public void startShortShooter() {
      this.velocity = shortvelocity;
      this.hood = shortshootpos;
      startShooter();
   }

   public boolean shoot(){
      if(this.isVelocityReached()){
         ramp.setPosition(shootRampPosition);
         return true;
      }
      ramp.setPosition(0);
      return false;
   }

   public void resetRamp(){
      ramp.setPosition(0);
   }

   public void setShooter(double distance) {

   }
}

