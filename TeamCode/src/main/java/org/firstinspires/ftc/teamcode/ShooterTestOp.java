package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "lolshooter")
public class ShooterTestOp extends OpMode {
    //declare motors
    private DcMotorEx frMotor, flMotor, brMotor, blMotor;
    private DcMotor intakeMotor, magazineMotor, shooterMotor, verticalArm;
    private CRServo horizontalArm;
    private DcMotorEx shooterMotorEx;

    // Constants
    private boolean isXPressed = false;
    private boolean isBPressed = false;


    private final double ROTATION = 0.4;
    private final double ARM_POWER = 0.2;
    private final double HIGH_SHOT_POWER = 5000;
    private final double MEDIUM_SHOT_POWER = 1000;
    private final double POWER_SHOT_POWER = 1200;
    private final double MAGAZINE_POWER = 1.0;
    private final double INTAKE_POWER = 1.0;
    private float currentShooterPower = 0.0f;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
        // Initialize hardware
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        horizontalArm = hardwareMap.get(CRServo.class, "hArm");
        verticalArm = hardwareMap.get(DcMotor.class, "vArm");
        verticalArm.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motors to brake when they have no power
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotorEx = (DcMotorEx) shooterMotor;

        //Declare motor directions
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //runtime.reset();
    }

    @Override
    public void loop() {
        //Driver 1
        double forwardController = gamepad1.left_stick_y;
        double strafeController = gamepad1.left_stick_x;
        double rotationController = gamepad1.right_stick_x;

        boolean armUp = gamepad1.left_bumper;
        float armDown = gamepad1.left_trigger;

        float armClose = gamepad1.right_trigger;
        boolean armOpen = gamepad1.right_bumper;

        //Driver 2
        boolean isIntakePress = gamepad2.left_bumper;
        boolean isOuttakePress = gamepad2.right_bumper;
        float magazineUp = gamepad2.left_trigger;
        float magazineDown = gamepad2.right_trigger;

        boolean powerShot = gamepad2.a;
        boolean mediumShot = gamepad2.x;
        boolean highShot = gamepad2.b;


        //Arm Code

        // If arm up is pressed and arm down is not, move arm up
        if (armUp && armDown == 0) verticalArm.setPower(0.7);

        // If arm down is pressed and arm up is not, move arm down
        else if (armDown > 0 && !armUp) verticalArm.setPower(-0.4);

        // Neither button is pressed, idle the arm
        else verticalArm.setPower(0);

        // If arm close is pressed and arm open is not, close arm
        if (armClose > 0 && !armOpen) horizontalArm.setPower(1);

        // If arm open is pressed and arm close is not, open arm
        else if (armOpen && armClose == 0) horizontalArm.setPower(-1);

        // Neither button is pressed, idle the arm
        else horizontalArm.setPower(0);

        // If move arm up slowly is pressed and slowly down is not, slowly move the arm up
        if (gamepad1.dpad_up && !gamepad1.dpad_down) verticalArm.setPower(0.25);

        // If move arm down slowly is pressed and slowly up is not, slowly move the arm down
        if (!gamepad1.dpad_up && gamepad1.dpad_down) verticalArm.setPower(-0.1);

        //Add telemetry to show velocity
        telemetry.addData("shooter-vel", shooterMotorEx.getVelocity());
        telemetry.update();

        // Different
        if (highShot && !mediumShot && !powerShot) {
            shooterMotorEx.setVelocity(HIGH_SHOT_POWER);
        } else if (!highShot && mediumShot && !powerShot) {
            shooterMotorEx.setVelocity(MEDIUM_SHOT_POWER);
        } else if (!highShot && !mediumShot && powerShot) {
            shooterMotorEx.setVelocity(POWER_SHOT_POWER);
        } else {
            shooterMotorEx.setVelocity(0.0);
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

        //Small strafe adjustments
        if (!gamepad1.dpad_left && !gamepad1.dpad_right)
            drivetrain(forwardController, strafeController, rotationController);
        else if(gamepad1.dpad_left && !gamepad1.dpad_right)
            drivetrainSlow(0, 1, 0.03);
        else if(!gamepad1.dpad_left && gamepad1.dpad_right)
            drivetrainSlow(0, -1, -0.03);

        //Power shot adjusments


        /*
        if (gamepad1.b && isBPressed == false){
            runtime.reset();
            if (runtime.seconds() > 0 && runtime.seconds() < 3){
                drivetrain(0, -1, 0);
            }
            drivetrain(0,0,0);
            isBPressed = true;
        }
        if (!gamepad1.b){
            isBPressed = false;
        }

         */




    }

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

    // Convert drive vector to motor output
    private void drivetrainSlow(double forward, double strafe, double rotation) {
        double frPower = forward - strafe - rotation;
        double flPower = forward + strafe + rotation;
        double brPower = forward + strafe - rotation;
        double blPower = forward - strafe + rotation;

        // Divide all power by the greatest absolute value to maintain proportionality while not exceeding +/- 1
        frMotor.setVelocity(frPower *500);
        flMotor.setVelocity(flPower *500);
        brMotor.setVelocity(brPower *500);
        blMotor.setVelocity(blPower *500);
    }



}