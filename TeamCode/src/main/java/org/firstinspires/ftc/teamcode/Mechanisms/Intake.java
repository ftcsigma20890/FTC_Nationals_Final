package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public  class  Intake {
   public static double fastPower = 1;
   public static double slowPower = 0.6;
   public static double reversePower = -1;

   DcMotor motor;

   public Intake(HardwareMap hardwareMap) {
      motor = hardwareMap.get(DcMotor.class, "intake");
      motor.setDirection(DcMotorSimple.Direction.REVERSE);
   }

   public void startIntake() {
      motor.setPower(fastPower);
   }

   public void slowIntake() {
      motor.setPower(slowPower);

   }

   public void stopIntake() {
      motor.setPower(0);
   }

   public void reverse(){
      motor.setPower(reversePower);
   }
}
