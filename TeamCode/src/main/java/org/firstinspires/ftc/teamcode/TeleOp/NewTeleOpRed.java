package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.Mechanisms.ColorCommand;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class NewTeleOpRed extends LinearOpMode {
    public static double strafePower = 1.1;
    public static double longShootPos = 0;
    public static double shortShootPos = 0;
    public static double rampPosition = 0.09;
    public static long delay = 350;
    Follower follower;
    Shooter shooter;
    Servo ramp;
    Servo hoodServo;
    Mechanisms mechanisms;
    boolean isRampUp = false;
    RevColorSensorV3 intakeColorSensor;
    ElapsedTime IntakeTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        mechanisms = new Mechanisms(hardwareMap, telemetry);
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");
        follower = Constants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap);
        ramp = hardwareMap.get(Servo.class, "ramp");
        ramp.setPosition(0);
        hoodServo = hardwareMap.get(Servo.class, "Hood2");
        hoodServo.setPosition(0);
        follower.startTeleopDrive(true);
        follower.update();
        mechanisms.pipelineSwitch(2);
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
            if (intakeColorSensor.getDistance(DistanceUnit.CM) < 5) {
                if (IntakeTimer.milliseconds() > 1 && IntakeTimer.milliseconds() < 2000) {
                    gamepad1.rumble(50);

                }
            } else {
                IntakeTimer.reset();
            }

            if (gamepad2.left_bumper) {
                mechanisms.startIntake();
                ramp.setPosition(0);
            } else if (gamepad2.a) {
                if (!isRampUp) {
                    ramp.setPosition(rampPosition);
                    sleep(delay);
                    isRampUp = true;
                }
                mechanisms.startIntake();
            } else if (gamepad2.left_trigger > 0.25) {
                mechanisms.reverseIntake();
            } else {
                isRampUp = false;
                mechanisms.stopIntake();
            }

            if (shooter.isVelocityReached()) {
                gamepad2.rumble(100);
            }
            if (gamepad1.left_trigger > 0.25) {
                follower.setMaxPower(0.4);
            } else {
                follower.setMaxPower(1);
            }
            telemetry.addData("IntakeTimer: ", IntakeTimer.milliseconds());
            telemetry.addData("Distance: ", intakeColorSensor.getDistance(DistanceUnit.CM));
            follower.setTeleOpDrive(-gamepad1.left_stick_y * 0.9, -gamepad1.left_stick_x * 0.9, -gamepad1.right_stick_x * 0.367, true);
            follower.update();
            mechanisms.update();

        }
    }
}