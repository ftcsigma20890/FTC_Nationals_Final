package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class TurretAlignment extends LinearOpMode {
   Follower follower;
   Limelight3A limelight3A;

   DcMotor turret;
   private double lastError = 0;

   @Override
   public void runOpMode() throws InterruptedException {
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(0,0,0));
      limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
      turret = hardwareMap.get(DcMotor.class, "turret");
      turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     waitForStart();
     while(opModeIsActive()){
        follower.update();
        align();
     }
   }

   public void align() {
      double currentHeading = (Math.toDegrees(follower.getHeading()) % 360) + 90;
      final double singleDegreeTicks = 2.4370;

      LLResult result = limelight3A.getLatestResult();
      if (result.isValid()) {

         double error = result.getTy();
         double derivative = error - lastError;

         double power = (0.17 * error) + (0.025 * derivative);

         power = (Math.max(-0.5, Math.min(0.5, power)));


         turret.setPower(-power);
         lastError = error;


      } else {
         turret.setTargetPosition((int) -(currentHeading * singleDegreeTicks));
         turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         turret.setPower(0.5);
      }
   }
}

