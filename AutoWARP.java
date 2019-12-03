package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// WARP Dec 2019

@Autonomous
public class AutoWARP extends LinearOpMode {
    DcMotor front_left_wheel;
    DcMotor back_left_wheel;
    DcMotor back_right_wheel;
    DcMotor front_right_wheel;

    Servo left_arm;
    Servo right_arm;

    ColorSensor left_color;
    ColorSensor right_color;
    DistanceSensor distance;

    @Override
    public void runOpMode() {
        // DC motors for holonomic drive.
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");
        // Arbitrary decision to reverse all -- could instead negate powers below.
        front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_right_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos for little arms
        left_arm = hardwareMap.servo.get("left_arm");
        right_arm = hardwareMap.servo.get("right_arm");
        left_arm.resetDeviceConfigurationForOpMode();
        right_arm.resetDeviceConfigurationForOpMode();
        left_arm.setDirection(Servo.Direction.FORWARD);
        right_arm.setDirection(Servo.Direction.REVERSE);

        // Sensors
        left_color = hardwareMap.colorSensor.get("left_color");
        right_color = hardwareMap.colorSensor.get("right_color");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        // wait for start button
        waitForStart();
        while (opModeIsActive()) {
            float reds[] = {left_color.red(), right_color.red()};
            float greens[] = {left_color.green(), right_color.green()};
            float blues[] = {left_color.blue(), right_color.blue()};
            float alphas[] = {left_color.alpha(), right_color.alpha()};

            telemetry.addData("Distance (cm)", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Red  ", reds);
            telemetry.addData("Green", greens);
            telemetry.addData("Blue ", blues);
            telemetry.addData("Alpha", alphas);
            telemetry.update();
        }
    }
}
