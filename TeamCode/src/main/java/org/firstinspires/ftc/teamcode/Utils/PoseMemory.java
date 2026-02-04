package org.firstinspires.ftc.teamcode.Utils;

import com.pedropathing.geometry.Pose;

public class PoseMemory {

   private static Pose lastPose;

   // Save pose values
   public static void savePose(Pose pose) {
      if (pose == null) return;
      lastPose = pose;
   }

   public static Pose getLastPose() {
      if (lastPose == null) return null;
      return lastPose;
   }

   public static double getLastY() {
      if (lastPose == null) {
         return 0;
      }
      return lastPose.getY();
   }

   public static double getLastX() {
      if (lastPose == null) {
         return 0;
      }
      return lastPose.getX();
   }

   public static double getLastHeading() {
      if (lastPose == null) {
         return 0;
      }
      return lastPose.getHeading();
   }

   public static boolean hasPose() {
      return lastPose != null;
   }
}