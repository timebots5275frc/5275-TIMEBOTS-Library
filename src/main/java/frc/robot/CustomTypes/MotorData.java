package frc.robot.CustomTypes;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class MotorData {
    private int ID;
    private int FREE_LIMIT;
    private int STALL_LIMIT;
    private PID pid;

    public MotorData(int ID, int FREE_LIMIT, int STALL_LIMIT, PID pid) {
        this.ID = ID;
        this.FREE_LIMIT = FREE_LIMIT;
        this.STALL_LIMIT = STALL_LIMIT;
        this.pid = pid;
    }

    public int getID() {return ID;}
    public int getFreeLimit() {return FREE_LIMIT;}
    public int getStallLimit() {return STALL_LIMIT;}
    public PID getPID() {return pid;}

    public void setID(int id) {this.ID = id;}
    public void setStallLimit(int stallLimit) {this.STALL_LIMIT = stallLimit;}
    public void setFreeLimit(int freeLimit) {this.FREE_LIMIT = freeLimit;}
    public void setPID(PID pid) {this.pid = pid;}

    /** Current limit + idle mode only (used for followers). */
    public <C extends SparkBaseConfig> C applyBaseTo(C config) {
        config.smartCurrentLimit(STALL_LIMIT, FREE_LIMIT);
        config.idleMode(IdleMode.kBrake);
        return config;
    }

    /** Everything a leader / standalone motor needs: limits, idle mode, and gains. */
    public <C extends SparkBaseConfig> C applyTo(C config) {
        applyBaseTo(config);
        pid.applyTo(config);
        return config;
    }
}