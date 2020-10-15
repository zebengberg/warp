package org.firstinspires.ftc.teamcode.secondarycode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Random;


@Autonomous(name = "MiniBotRoomba", group = "MiniBot")
public class MiniBotRoomba extends LinearOpMode {

    private DcMotor left_motor, right_motor;
    private DistanceSensor left_distance, right_distance;
    private Servo left_arm, right_arm;


    @Override
    public void runOpMode() {

        left_motor = hardwareMap.dcMotor.get("left_motor");
        right_motor = hardwareMap.dcMotor.get("right_motor");
        DcMotor[] motors = new DcMotor[] {left_motor, right_motor};
        for (DcMotor motor : motors) {
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        left_distance = hardwareMap.get(DistanceSensor.class, "left_distance");
        right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");

        left_arm = hardwareMap.servo.get("left_arm");
        right_arm = hardwareMap.servo.get("right_arm");
        left_arm.setDirection(Servo.Direction.REVERSE);
        right_arm.setDirection(Servo.Direction.FORWARD);
        left_arm.setPosition(0.0);
        right_arm.setPosition(0.0);

        waitForStart();
        while (opModeIsActive()) {
            explore();
        }
    }

    private void printStatus() {
        telemetry.addData("left motor power", left_motor.getPower());
        telemetry.addData("right motor power", right_motor.getPower());
        telemetry.addData("left motor position", right_motor.getCurrentPosition());
        telemetry.addData("right motor position", right_motor.getCurrentPosition());
        telemetry.addData("left distance", left_distance.getDistance(DistanceUnit.CM));
        telemetry.addData("right distance", right_distance.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    private int getRandom(int min, int max) {
        Random r = new Random();
        return r.nextInt(max - min) + min;
    }

    private double getDistance() {
        return Math.min(left_distance.getDistance(DistanceUnit.CM), right_distance.getDistance(DistanceUnit.CM));
    }

    private void randomCW() {
        int start = left_motor.getCurrentPosition();
        int rand = getRandom(500, 2000);
        while (left_motor.getCurrentPosition() < start + rand) {
            left_motor.setPower(1);
            right_motor.setPower(-1);
            printStatus();
        }
    }

    private void randomCCW() {
        int start = right_motor.getCurrentPosition();
        int rand = getRandom(500, 2000);
        while (right_motor.getCurrentPosition() < start + rand) {
            right_motor.setPower(1);
            left_motor.setPower(-1);
            printStatus();
        }
    }

    private void goForward(int x) {
        int start = right_motor.getCurrentPosition();
        while ((right_motor.getCurrentPosition() < start + x) && (getDistance() > 50)) {
            right_motor.setPower(1);
            left_motor.setPower(1);
            printStatus();
        }
        right_motor.setPower(0);
        left_motor.setPower(0);
    }

    private void goBack(int x) {
        int start = right_motor.getCurrentPosition();
        while (right_motor.getCurrentPosition() > start - x) {
            right_motor.setPower(-0.5);
            left_motor.setPower(-0.5);
            printStatus();
        }
        right_motor.setPower(0);
        left_motor.setPower(0);
    }

    private void claw() {
        left_arm.setPosition(0.5);
        sleep(500);
        right_arm.setPosition(0.5);
        left_arm.setPosition(0.0);
        sleep(500);
        right_arm.setPosition(0.0);
    }

    private void explore() {
        int rand = getRandom(2000, 10000);
        goForward(rand);
        rand = getRandom(200, 400);
        goBack(rand);
        rand = getRandom(0, 2);
        if (rand == 0) {
            randomCCW();
        } else {
            randomCW();
        }
    }
}


