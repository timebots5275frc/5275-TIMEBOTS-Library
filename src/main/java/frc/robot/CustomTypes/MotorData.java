package frc.robot.CustomTypes;


public class MotorData {
    private int ID;
    private int FREE_LIMIT;
    private int STALL_LIMIT;
    private PID pid;

    public MotorData(int ID, int FREE_LIMIT, int STALL_LIMIT, PID pid){ // defines all constants with a motor){
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
}
