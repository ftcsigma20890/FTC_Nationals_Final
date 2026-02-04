package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {
   public static double MAX_DEGREES = 300;
   public static double TICKS_PER_REV = 537.6;
   public static double GEAR_RATIO = 1.0;
   public static double MOTOR_POWER = 0.3;
   public static int turretPosition = 0;
   public static double kp = 0.027;
   public static double kd = 0.04;
   public static double maxPower = 1;
   public static double turretInitPower = 0.3;
   public static double turretOffset = 0;
   double minticklimit = -500;
   double maxtickslimit = 500;
   int turretZero = 0;
   private DcMotorEx turretMotor;
   private DigitalChannel magneticSensor;
   private double targetDegrees = 0;
   // PD state
   private double lastError = 0;

   public Turret(HardwareMap hardwareMap) {
      turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
      turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      turretMotor.setPower(0);
//        while (!magneticSensor.getState()) {
//            turretMotor.setPower(turretInitPower);
//        }
//        maxtickslimit = tu  rretMotor.getCurrentPosition();
//        while (!magneticSensor.getState()) {
//            turretMotor.setPower(-turretInitPower);
//        }
//        minticklimit = turretMotor.getCurrentPosition();

   }


   public double getTicks() {
      return turretMotor.getCurrentPosition();
   }

   public void setTurretTicks(int ticks) {
      turretMotor.setTargetPosition(ticks);
      turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      turretMotor.setPower(1);
   }

   public boolean isBusy() {
      return turretMotor.isBusy();
   }

   public boolean isAligned(double tx) {
      return Math.abs(tx) < 1;
   }

   public double alignLimeLight(double tx) {
      double error = tx;
      double derivative = error - lastError;

      double power = (kp * error) + (kd * derivative);

      power = (Math.max(-maxPower, Math.min(maxPower, power)));
      if (!turretMotor.isBusy()) {
         turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      }

      if (power > 0) {
         if (turretMotor.getCurrentPosition() + turretZero < minticklimit) {
            turretMotor.setPower(0);
            return 0;
         }
      } else if (power < 0) {
         if (turretMotor.getCurrentPosition() + turretZero > maxtickslimit) {
            turretMotor.setPower(0);
            return 0;
         }

      }

      turretMotor.setPower(-power);
      lastError = error;
      return power;
   }

   public void setPower(double power) {
      turretMotor.setPower(power);
   }


   public void stop() {
      turretMotor.setPower(0);
      lastError = 0;
   }


}


