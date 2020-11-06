package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double WHEEL_DIAMETER = 75.0;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final int TICKS_PER_ROTATION = 28;
    public static final int GEAR_RATIO = 40;
    public static final int TICKS_PER_METER = (int) (TICKS_PER_ROTATION * GEAR_RATIO * WHEEL_CIRCUMFERENCE);
}
