package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "lolpid")
public class PIDTestOp extends OpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor, liftMotor;
    private CRServo horizontalArm;

    // Constants
    private final float ROTATION = 0.4f;
    private final float ARM_POWER = 0.2f;
    private final float HIGH_SHOT_POWER = 1.0f;
    private final float MEDIUM_SHOT_POWER = 0.43f;
    private final float POWER_SHOT_POWER = 0.5f;
    private final float MAGAZINE_POWER = 1.0f;
    private final float INTAKE_POWER = 1.0f;

    private float currentShooterPower = 0.0f;

    private int lastCounts = 0;
    private long lastTime = 0;

    @Override
    public void init() {
        // Initialize hardware
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        liftMotor = hardwareMap.get(DcMotor.class, "vArm");
        horizontalArm = hardwareMap.get(CRServo.class, "hArm");
        horizontalArm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to brake when they have no power
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //runtime.reset();
    }

    @Override
    public void loop() {
        //Driver 1
        double forwardController = gamepad1.left_stick_y;
        double strafeController = gamepad1.left_stick_x;
        double rotationController = gamepad1.right_stick_x;

        boolean armUp = gamepad1.right_bumper;
        float armDown = gamepad1.right_trigger;

        boolean armClose = gamepad1.left_bumper;
        float armOpen = gamepad1.left_trigger;

        //Driver 2
        boolean isIntakePress = gamepad2.left_bumper;
        boolean isOuttakePress = gamepad2.right_bumper;
        float magazineUp = gamepad2.left_trigger;
        float magazineDown = gamepad2.right_trigger;

        boolean powerShot = gamepad2.a;
        boolean mediumShot = gamepad2.x;
        boolean highShot = gamepad2.b;

        boolean powerLift = gamepad1.a;


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

        if (armUp && armDown == 0 && !powerLift) {
            liftMotor.setPower(0.2);
        } else if (armDown > 0 && !armUp && !powerLift) {
            liftMotor.setPower(-0.2);
        } else if (armDown == 0 && !armUp && powerLift){
            liftMotor.setPower(0.6);
        }
        else {
            liftMotor.setPower(0);
        }

        if (armClose && armOpen == 0) {
            horizontalArm.setPower(0.3);
        } else if (armOpen > 0 && !armClose) {
            horizontalArm.setPower(-0.3);
        } else {
            horizontalArm.setPower(0);
        }

        int lastCounts = this.lastCounts;
        long lastTime = this.lastTime;

        this.lastCounts = shooterMotor.getCurrentPosition();
        this.lastTime = System.currentTimeMillis();

        float deltaCounts = (float) this.lastCounts - (float) lastCounts;
        float deltaTime = (float) this.lastTime - (float) lastTime;

        float shooterVelocity = deltaCounts / deltaTime;
        telemetry.addData("vel", shooterVelocity);
        telemetry.update();

        // shooter logic
        if (highShot && !mediumShot && !powerShot) {
            shooterMotor.setPower(HIGH_SHOT_POWER);
        } else if (!highShot && mediumShot && !powerShot) {
            shooterMotor.setPower(MEDIUM_SHOT_POWER);
        } else if (!highShot && !mediumShot && powerShot) {
            shooterMotor.setPower(POWER_SHOT_POWER);
        } else {
            shooterMotor.setPower(0.0);
        }


        // magazine logic
        double magazinePower;

        if (magazineUp > 0.2 && magazineDown == 0) {
            magazinePower = MAGAZINE_POWER;
        } else if (magazineUp == 0.0 && magazineDown > 0.2) {
            magazinePower = -MAGAZINE_POWER;
        } else {
            magazinePower = 0.0;
        }

        magazineMotor.setPower(magazinePower);


        // intake controls
        if (isIntakePress) {
            intakeMotor.setPower(-INTAKE_POWER);
        } else if (isOuttakePress) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0);
        }


        drivetrain(forwardController, strafeController, rotationController);
    }

    // Convert drive vector to motor output
    private void drivetrain(double forward, double strafe, double rotation) {
        double frPower = forward - strafe - rotation;
        double flPower = forward + strafe + rotation;
        double brPower = forward + strafe - rotation;
        double blPower = forward - strafe + rotation;

        // Find the power with the greatest absolute value
        double maxPower = Math.max(Math.abs(frPower), Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));

        // Divide all power by the greatest absolute value to maintain proportionality while not exceeding +/- 1
        frMotor.setPower(frPower / maxPower);
        flMotor.setPower(flPower / maxPower);
        brMotor.setPower(brPower / maxPower);
        blMotor.setPower(blPower / maxPower);
    }


}
