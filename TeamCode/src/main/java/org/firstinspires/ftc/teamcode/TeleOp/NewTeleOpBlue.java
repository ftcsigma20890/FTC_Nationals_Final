package org.firstinspires.ftc.teamcode.TeleOp;

import android.media.AudioManager;
import android.media.ToneGenerator;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.teamcode.Mechanisms.ColorCommand;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp
public class NewTeleOpBlue extends LinearOpMode {
    public static double strafePower = 0.7;
    public static double longShootPos = 0;
    public static double shortShootPos = 0;
    public static double rampPosition = 0.09;
    DcMotor lf, lr, rf, rr;
    Follower follower;
    Shooter shooter;
    Servo ramp;
    Servo hoodServo;
    Mechanisms mechanisms;

    RevColorSensorV3 intakeColorSensor;
    ElapsedTime IntakeTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");
        mechanisms = new Mechanisms(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap);
        ramp = hardwareMap.get(Servo.class, "ramp");
        ramp.setPosition(0);
        hoodServo = hardwareMap.get(Servo.class, "Hood2");
        hoodServo.setPosition(0);
        follower.startTeleopDrive(false);
        follower.update();
        mechanisms.pipelineSwitch(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_down) {
                shooter.startShortShooter();
                hoodServo.setPosition(shortShootPos);
            } else if (gamepad2.dpad_up) {
                shooter.startLongShooter();
                hoodServo.setPosition(longShootPos);
            } else if (gamepad2.right_bumper) {
                shooter.startShooter();
            } else {
                shooter.stopShooter();
            }
            if (gamepad2.left_bumper) {
                mechanisms.startIntake();
                ramp.setPosition(0);
            } else if (gamepad2.a) {
                mechanisms.startIntake();
                ramp.setPosition(rampPosition);
            } else if (gamepad2.left_trigger > 0.25) {
                mechanisms.reverseIntake();
            } else {
                mechanisms.stopIntake();
            }

            if (intakeColorSensor.getDistance(DistanceUnit.CM) < 5) {
                if (IntakeTimer.milliseconds() > 1 && IntakeTimer.milliseconds() < 2000) {
                    gamepad1.rumble(50);

                }
            } else {
                IntakeTimer.reset();
            }

            if (gamepad2.yWasPressed()) {
                mechanisms.setTurretTicks(0);
            }


            if (gamepad2.yWasPressed()) {
                mechanisms.shoot();
            }

            if (shooter.isVelocityReached()) {
                gamepad2.rumble(100);

            }
//         if (colorCommand.ledState==true){
//            gamepad2.rumble(2500);
//         }
            if (gamepad1.right_trigger > 0.25) {
                follower.setMaxPower(0.4);
            } else {
                follower.setMaxPower(1);
            }
            follower.setTeleOpDrive(-gamepad1.left_stick_y * 0.9, -gamepad1.left_stick_x * 0.9, -gamepad1.right_stick_x * 0.67, true);
//
            telemetry.addData("IntakeTimer: ", IntakeTimer.milliseconds());
            telemetry.addData("Distance: ", intakeColorSensor.getDistance(DistanceUnit.CM));
            follower.update();
            mechanisms.update();
        }

    }
}
