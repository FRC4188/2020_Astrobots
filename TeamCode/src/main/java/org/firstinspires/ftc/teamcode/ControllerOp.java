package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "lol")
public class ControllerOp extends OpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor;
    private CRServo arm;

    private long lastTime = 0;
    private int lastPosition = 0;

    @Override
    public void init() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(CRServo.class, "wobbleArm");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //runtime.reset();
    }

    @Override
    public void loop() {
        //Driver 1
        double forwardController = gamepad1.left_stick_y;
        double strafeController = gamepad1.left_stick_x;
        double rotationController = gamepad1.right_stick_x;

        boolean armLeft = gamepad1.left_bumper;
        boolean armRight = gamepad1.right_bumper;

        //Driver 2
        boolean isIntakePress = gamepad2.left_bumper;
        boolean isOuttakePress = gamepad2.right_bumper;
        float magazineUp = gamepad2.left_trigger;
        float magazineDown = gamepad2.right_trigger;

        boolean powerShot = gamepad2.a;
        boolean mediumShot = gamepad2.x;
        boolean highShot = gamepad2.b;
        /*
        // calc vel
        long lastTime = this.lastTime;
        int lastPosition = this.lastPosition;

        this.lastTime = System.currentTimeMillis();
        this.lastPosition = shooterMotor.getCurrentPosition();

        double deltaPosition = (double) this.lastPosition - (double) lastPosition;
        double deltaTime = (double) this.lastTime - (double) lastTime;

        telemetry.addData("shooter-vel", deltaPosition / deltaTime);
        telemetry.addData("delta-position", deltaPosition);
        telemetry.addData("delta-time", deltaTime);

        telemetry.update();
        */

        //servo arm
        if (armLeft && !armRight) {
            arm.setPower(0.2);
        } else if (armRight && !armLeft) {
            arm.setPower(-0.2);
        } else {
            arm.setPower(0);
        }
        //shooter


        if (highShot && !mediumShot && !powerShot) {
            shooterMotor.setPower(1.0);
        }
        else if (!highShot && mediumShot && !powerShot) {
            shooterMotor.setPower (0.5);
        }
        else if (!highShot && !mediumShot && powerShot){
            shooterMotor.setPower (0.7);
        }
        else {
            shooterMotor.setPower (0.0);
        }



        //magazine
        double magazinePower;

        if (magazineUp > 0.2 && magazineDown == 0) {
            magazinePower = 1.0;
        }
        else if (magazineUp == 0.0 && magazineDown > 0.2){
            magazinePower = -1.0;
        }
        else {
            magazinePower = 0.0;
        }

        magazineMotor.setPower(magazinePower);



        //intake
        if (isIntakePress) {
            intakeMotor.setPower(-1.0);
        } else if (isOuttakePress) {
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0);
        }



        drivetrain(forwardController, strafeController, rotationController);
    }

    private void drivetrain(double forward, double strafe, double rotation) {
        double frPower = forward - strafe - rotation;
        double flPower = forward + strafe + rotation;
        double brPower = forward + strafe - rotation;
        double blPower = forward - strafe + rotation;

        double maxPower = Math.max(Math.abs(frPower), Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));

        frMotor.setPower(frPower / maxPower);
        flMotor.setPower(flPower / maxPower);
        brMotor.setPower(brPower / maxPower);
        blMotor.setPower(blPower / maxPower);
    }





}
