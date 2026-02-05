package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Wait;

public class Mechanisms {
   Intake intake;
   Shooter shooter;
   Vision vision;
   TelemetryManager telemetryManager;
   Telemetry telemetry;
   Turret turret;

   boolean shootingStarted = false;
   ElapsedTime shootingTime = new ElapsedTime();
   public Mechanisms(HardwareMap hardwareMap, Telemetry telemetry) {
      intake = new Intake(hardwareMap);
      shooter = new Shooter(hardwareMap);
      vision = new Vision(hardwareMap);
      turret = new Turret(hardwareMap);
      this.telemetry = telemetry;
      telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
   }

   public void startIntake() {
      intake.startIntake();
   }

   public void slowIntake() {
      intake.slowIntake();
   }

   public void stopIntake() {
      intake.stopIntake();
   }

   public void reverseIntake() {
      intake.reverse();
   }


   public void startShooter() {
      shooter.startShooter();
   }

   public void shoot(){
      while(!shooter.shoot()){
         shooter.startShooter();
      };
      intake.startIntake();
      Wait.mySleep(1500);
      shooter.resetRamp();
      intake.stopIntake();
   }

   public void startShortShooter() {
      shooter.startShortShooter();
   }
   public void startLongShooter() {
      shooter.startLongShooter();
   }

   public void stopShooter(){
      shooter.stopShooter();
   }

   public void pipelineSwitch(int index) {
      vision.pipelineSwitch(index);
   }

   public double getTurretTicks() {
      return turret.getTicks();
   }
   public void setTurretPower(double power){
      turret.setPower(power);
   }

   public boolean isTurretAligned(){
      return vision.isValid() && Math.abs(vision.getTy()) < 1.2;
   }

   public boolean isTurretBusy(){
      return turret.isBusy();
   }

   public void setTurretTicks(int ticks) {
      turret.setTurretTicks(ticks);
   }


   public void update() {
      if (vision.isValid()) {
         shooter.setShooter(vision.getDistance());
         turret.alignLimeLight(vision.getTy());

      } else {
         turret.setPower(0);
      }

      telemetryManager.addData("Current Velocity", shooter.getVelocity());
      telemetryManager.addData("Target Velocity", shooter.getTargetVelocity());
      telemetryManager.addData("Turret Ticks", turret.getTicks());
      telemetryManager.addData("isVelocityReached", shooter.isVelocityReached());
      telemetryManager.update(telemetry);
   }

}
