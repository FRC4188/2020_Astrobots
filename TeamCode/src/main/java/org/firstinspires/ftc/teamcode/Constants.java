package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double WHEEL_DIAMETER = 0.075;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final int TICKS_PER_ROTATION = 960;
    public static final int GEAR_RATIO = 1;
    public static final double TICKS_PER_METER = (TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE * GEAR_RATIO);
}
