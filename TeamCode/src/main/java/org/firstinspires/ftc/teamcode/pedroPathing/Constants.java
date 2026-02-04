package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
   public static double LATERAL_MULTIPLIER = 1.30;
   public static FollowerConstants followerConstants = new FollowerConstants().mass(12)
      .forwardZeroPowerAcceleration(-27.9)
      .lateralZeroPowerAcceleration(-70.2682)
      .translationalPIDFCoefficients(new PIDFCoefficients(0.5,0,0.05,0))
      .headingPIDFCoefficients(new PIDFCoefficients(1.5,0,0.12,0.05))
      .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05, 0.00001, 0.00008, 0.6, 0.01))
   ;


   public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
   public static MecanumConstants mecanumConstants = new MecanumConstants()
      .maxPower(1)
      .rightFrontMotorName("RF")
      .rightRearMotorName("RR")
      .leftFrontMotorName("LF")
      .leftRearMotorName("LR")
      .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
      .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
      .xVelocity(66.26466)
      .yVelocity(28.58);


   public static PinpointConstants localizerConstants = new PinpointConstants()
      .hardwareMapName("PinPoints")
      .forwardPodY(2.3)
      .strafePodX(-5.2625)
      .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
      .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
      .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

   public static Follower createFollower(HardwareMap hardwareMap) {
      return new FollowerBuilder(followerConstants, hardwareMap)
         .pathConstraints(pathConstraints)
         .mecanumDrivetrain(mecanumConstants)
         .pinpointLocalizer(localizerConstants)
         .build();
   }
}
