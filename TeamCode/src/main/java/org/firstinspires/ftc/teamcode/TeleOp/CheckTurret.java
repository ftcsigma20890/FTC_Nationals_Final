package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;

@TeleOp
public class CheckTurret extends LinearOpMode {
   Mechanisms mechanisms;

   @Override
   public void runOpMode() throws InterruptedException {
      mechanisms = new Mechanisms(hardwareMap, telemetry);
      waitForStart();
      while (opModeIsActive()) {
         mechanisms.update();
      }
   }
}
