package org.firstinspires.ftc.potencode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.potencode.utils.ButtonState;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="JebbyOp")
public class JebbyOp extends OpMode {
    private Jeb jeb;

    private ButtonState backButtonToggle;
    private ButtonState rightBumperToggle;
    private ButtonState leftBumperToggle;
    private ButtonState gamepad2y;

    private boolean intakeMoving;
    private double driveSpeedModifier;

    private static double driveX;
    private static double driveY;
    private static double driveTurn;

    private double slideY;
    private int targetArmPos = 0;

    private ElapsedTime motorHoldTime = new ElapsedTime();

    @Override
    public void init() {
        jeb = new Jeb(hardwareMap, telemetry);
        jeb.awake();

        jeb.resetDriveEncoders();
        jeb.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jeb.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jeb.frontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jeb.backMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jeb.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jeb.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backButtonToggle = new ButtonState();
        rightBumperToggle = new ButtonState();
        leftBumperToggle = new ButtonState();
        gamepad2y = new ButtonState();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        motorHoldTime.reset();
    }

    @Override
    public void loop() {
        // drive
        backButtonToggle.update(gamepad1.back);
        telemetry.addData("FOD", backButtonToggle.buttonState);

        driveSpeedModifier = Consts.DEFAULT_DRIVE_POWER - gamepad1.right_trigger * Consts.DEFAULT_DRIVE_POWER;
        // Allow turbo mode only if slide is not too high
        if (jeb.slideMotor.getCurrentPosition() < Consts.MID_ARM_POS) {
            driveSpeedModifier += gamepad1.left_trigger * (1 - Consts.DEFAULT_DRIVE_POWER);
        }
        driveSpeedModifier = Range.clip(driveSpeedModifier,  Consts.MIN_DRIVE_POWER, 1);
        telemetry.addData("Move speed modifier", driveSpeedModifier);

        driveX = (Math.pow(-gamepad1.left_stick_x, 3) * driveSpeedModifier) + (-gamepad2.right_stick_x * Consts.ARM_DRIVE_POWER);
        driveY = (Math.pow(-gamepad1.left_stick_y, 3) * driveSpeedModifier) + (-gamepad2.right_stick_y * Consts.ARM_DRIVE_POWER);
        driveTurn = -Math.pow(gamepad1.right_stick_x, 3) * driveSpeedModifier;


        if (gamepad1.x) {
            jeb.resetAngle();
        }
        if (gamepad1.y) {
            if (Math.abs(driveX) > Math.abs(driveY)) {
                driveY = 0;
            } else {
                driveX = 0;
            }
        }

        if (backButtonToggle.buttonState) {
            jeb.driveFOD(driveX, driveY, driveTurn);
        }
        else {
            jeb.driveVelocity(driveX, driveY, driveTurn);
        }

        // arm

        gamepad2y.update(gamepad2.y);
        slideY = -gamepad2.left_stick_y;
        if (gamepad2y.buttonState) {
            slideY = slideY * -1;
        }

        if (gamepad2.dpad_down) targetArmPos = Consts.PICKUP_ARM_POS;
        else if (gamepad2.dpad_left) targetArmPos = Consts.LOW_ARM_POS;
        else if (gamepad2.dpad_right) targetArmPos = Consts.MID_ARM_POS;
        else if (gamepad2.dpad_up) targetArmPos = Consts.HIGH_ARM_POS;
        else if (gamepad2.right_stick_y == 0 && targetArmPos == 0) targetArmPos = jeb.slideMotor.getCurrentPosition();

        telemetry.addData("slideY", slideY);
        telemetry.addData("limit pressed", jeb.limitSlide.isPressed());
        telemetry.addData("motor hold time", motorHoldTime);
        telemetry.addData("slide pos", jeb.slideMotor.getCurrentPosition());

        if (gamepad2.b && (!jeb.limitSlide.isPressed())) {
            telemetry.addLine("B Pressed! Full driver control, be careful going up!!!");
            jeb.slideMotor.setPower(slideY);
        }
        else {
            if (gamepad2.left_stick_y != 0) {
                motorHoldTime.reset();
                targetArmPos = 0;
                jeb.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (jeb.limitSlide.isPressed()) {
                    jeb.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    jeb.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    jeb.slideMotor.setPower(0);
                }
                if (jeb.slideMotor.getCurrentPosition() <= Consts.MIN_ARM_SLIDE_POS) {
                    jeb.slideMotor.setPower(0);
                }
                if ((!jeb.limitSlide.isPressed() || slideY > 0) &&
                        (jeb.slideMotor.getCurrentPosition() > Consts.MIN_ARM_SLIDE_POS || slideY > 0)) {
                    if (jeb.slideMotor.getCurrentPosition() > Consts.PICKUP_ARM_POS || slideY < 0) {
                        jeb.slideMotor.setPower(Math.pow(slideY, 1));
                    } else {
                        jeb.slideMotor.setPower(Math.pow(slideY, 1)*0.5);
                    }
                }
            }
            else {
                if (motorHoldTime.seconds() > 15) {
                    telemetry.addLine("shutting motor down");
                    jeb.slideMotor.setPower(0);
                }
                else {
                    jeb.holdMotor(jeb.slideMotor, targetArmPos, Consts.SLIDE_VEL);
                }
            }
        }




        // claw
        rightBumperToggle.update(gamepad2.right_bumper);
        // test program
        /*if (gamepad2.left_bumper || gamepad2.right_bumper) {
            if (!intakeMoving and distance sensor sees nothing){
                intakeMoving = true; //intake grabs cone
                jeb.clawServoA.setPower(Consts.DEFAULT_ARM_POWER);
                jeb.clawServoB.setPower(-Consts.DEFAULT_ARM_POWER);
            } else if (!intakeMoving and distance sensor sees something) {
                intakeMoving = true; //intake lets go of cone
                jeb.clawServoA.setPower(-Consts.DEFAULT_ARM_POWER);
                jeb.clawServoB.setPower(Consts.DEFAULT_ARM_POWER);
            }
        } else {
            intakeMoving = false;
            jeb.clawServoA.setPower(0);
            jeb.clawServoB.setPower(0);
        }*/

        telemetry.addData("distance from minion", jeb.intakeDistanceSensor.getDistance(DistanceUnit.MM));

        if ((gamepad2.left_bumper && gamepad2.right_bumper) ||
                ((gamepad2.left_bumper || gamepad2.right_bumper) &&
                        jeb.intakeDistanceSensor.getDistance(DistanceUnit.MM) > 65)) {
            jeb.clawServoA.setPower(-Consts.DEFAULT_ARM_POWER);
            jeb.clawServoB.setPower(Consts.DEFAULT_ARM_POWER);
        }
        else if (gamepad2.left_trigger > 0.4 || gamepad2.right_trigger > 0.4) {
            jeb.clawServoA.setPower(Consts.DEFAULT_ARM_POWER);
            jeb.clawServoB.setPower(-Consts.DEFAULT_ARM_POWER);
        }
        else {
            jeb.clawServoA.setPower(0);
            jeb.clawServoB.setPower(0);
        }

    }



}
