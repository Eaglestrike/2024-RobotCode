#include "Intake/Wrist.h"
#include <iostream>

//constructor j sets motor to brake mode
Wrist::Wrist(){
    UpdatePose();
    m_setPt = m_curPos;
    if (m_DBGstate != NONE || m_DBGstate != AUTO_TUNER){
        m_shuff = *(new ShuffleboardSender("Wrist", true));
    }
    // m_wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Wrist::Zero() {
    m_absEncoderInit = GetAbsEncoderPos();
    m_wristMotor.SetPosition(units::turn_t(0.0));
}

double Wrist::GetRelPos() {
    return -m_wristMotor.GetPosition().GetValueAsDouble()  * 2 * M_PI + m_absEncoderInit;
}

// absolute encoder pos in radians
double Wrist::GetAbsEncoderPos() {
    return m_wristEncoder.GetAbsolutePosition() * 2 * M_PI + WRIST_ABS_ENCODER_OFFSET;
}


// needs to be called INSTEAD of teleop periodic
void Wrist::ManualPeriodic(double wristVolts){
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -MAX_VOLTS, MAX_VOLTS)));
}

void Wrist::CorePeriodic(){
    UpdatePose();
}

// teleop periodic runs on state machine
void Wrist::CoreTeleopPeriodic(){
    if (m_DBGstate == TUNE_FFPID){
        m_shuff.update(true);
    } else if (m_DBGstate == POS_READER){
        m_shuff.update(false);
        return;
    } else if (m_DBGstate == CONST_VOLTAGE){
        m_shuff.update(true);
        SetVoltage();
        return;
    } else if (m_DBGstate == AUTO_TUNER){
        m_autoTuner.setPose({m_curPos, m_curVel, m_curAcc});
        double voltage = std::clamp(m_autoTuner.getVoltage(),-MAX_VOLTS, MAX_VOLTS);
        m_wristMotor.SetVoltage(units::volt_t(voltage));
        return;
    }
    
    double wristVolts = 0;

    switch (m_state){
        case MOVING:
            UpdateTargetPose(); // bc still using motion profile 
            wristVolts = FFPIDCalculate();
            if (AtSetpoint()){
                m_state = AT_TARGET;
                ResetPID();
                m_targetPos = m_setPt;
                m_targetVel = 0.0;
                m_targetAcc = 0.0;
            } 
            break;
        case AT_TARGET:
            wristVolts = FFPIDCalculate();
            break;
    }
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -MAX_VOLTS, MAX_VOLTS)));
}

void Wrist::MoveTo(double newpos){
    ChangeSetPt(newpos);
    MoveToSetPt();
}

void Wrist::ChangeSetPt(double newSetpt){
    m_newSetPt = std::clamp(newSetpt, MIN_POS, MAX_POS);
}

void Wrist::MoveToSetPt(){
    m_setPt = m_newSetPt;
    m_targetPos = m_curPos;
    m_targetVel = 0.0;
    m_targetAcc = MAX_ACC;
    if (m_setPt < m_curPos) m_targetAcc *= -1;
    ResetPID();
    CalcSpeedDecreasePos();
    m_state = MOVING;
}

//disables
void Wrist::Kill(){
    m_state = STOPPED;
}

Wrist::MechState Wrist::GetState(){
    return m_state;
}

// need to check
double Wrist::GetRelPos() {
    // steps to rads basically bc 2048 steps for a falcon
    return m_wristMotor.GetPosition().GetValueAsDouble() * 2 * M_PI;
}

//Updates the current position, velocity, and acceleration of the wrist
void Wrist::UpdatePose(){
    double newPos = GetRelPos(); // might need to negate or do some wrap around calculations
    double newVel = (newPos - m_curPos)/0.02;
    m_curAcc = (newVel - m_curVel)/0.02;
    m_curVel = newVel;
    m_curPos = newPos;
}

//updates the trapezoidal motion profile
void Wrist::UpdateTargetPose(){
    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;
    
    newP += m_targetVel * 0.02;

    if (m_speedDecreasePos < m_setPt){ // if trapezoid is pos
        if (newP > m_speedDecreasePos) // if after turn pt
            newV = std::max(0.0, m_targetVel - MAX_ACC * 0.02);
        else 
            newV = std::min(MAX_VEL, m_targetVel + MAX_ACC * 0.02);
    } else {
        if (newP > m_speedDecreasePos) // if before the turn pt
            newV = std::max(-MAX_VEL, m_targetVel - MAX_ACC * 0.02);
        else 
            newV = std::min(0.0, m_targetVel + MAX_ACC * 0.02);
    }

    if (newV-m_targetVel == 0) newA = 0;
    else if (newV > m_targetVel) newA = MAX_ACC;
    else newA = -MAX_ACC;

    m_targetPos = newP;
    m_targetVel = newV;
    m_targetAcc = newA;
}

//calculates voltage output with feedforwardPID
double Wrist::FFPIDCalculate(){
    double posErr = m_targetPos - m_curPos, 
    velErr = m_targetVel - m_curVel;
    m_totalErr += posErr * 0.02;
    if (fabs(posErr) <= POS_TOLERANCE) posErr =0;
    double pid = m_kp*posErr + m_kd*velErr + m_ki*m_totalErr;
    double s = m_s;
    if (m_targetVel < 0) s = -m_s;
    else if (m_targetVel == 0) s = 0;
    double ff = m_g * cos(m_targetPos) + s + m_v*m_targetVel + m_a*m_targetAcc;
    if (m_DBGstate == TUNE_FFPID){
        m_shuff.PutNumber("posErr", posErr); 
        m_shuff.PutNumber("velErr", velErr); 
        m_shuff.PutNumber("ff out", ff); 
        m_shuff.PutNumber("pid out", pid); 
        //see how each term is contributing to ff 
    }
    return pid+ff;
}

void Wrist::CoreShuffleboardInit(){
    m_shuff.PutString("DBG state", DBGToString());
    switch(m_DBGstate){
        case POS_READER:
            m_shuff.add("Pos", &m_curPos, false);
            m_shuff.add("Vel", &m_curVel, false);
            m_shuff.add("Acc", &m_curAcc, false);
            break;
        case CONST_VOLTAGE:
            m_shuff.add("Voltage", &m_voltReq, true);
            break;
        case TUNE_FFPID:
            m_shuff.add("g", &m_g, true);
            m_shuff.add("s", &m_s, true);
            m_shuff.add("v", &m_v, true);
            m_shuff.add("a", &m_a, true);
            m_shuff.add("p", &m_kp, true);
            m_shuff.add("i", &m_ki, true);
            m_shuff.add("d", &m_kd, true);
            m_shuff.add("SetPoint", &m_newSetPt, true);
            m_shuff.addButton("Deploy", [&]{MoveToSetPt();});

            m_shuff.add("cur pos", &m_curPos, false);
            m_shuff.add("cur vel", &m_curVel, false);
            m_shuff.add("cur acc", &m_curAcc, false);

            m_shuff.add("targ pos", &m_targetPos, false);
            m_shuff.add("targ vel", &m_targetVel, false);
            m_shuff.add("targ acc", &m_targetAcc, false);
            break;
    }
}

std::string Wrist::DBGToString(){
    switch(m_DBGstate){
        case POS_READER:
            return "POS READER";
        case CONST_VOLTAGE:
            return "CONST VOLTAGE";
        case TUNE_FFPID:
            return "TUNE FFPID";
    }
    return "";
}

//calculates the position at which the speed will begin decreasing
void Wrist::CalcSpeedDecreasePos(){
    double MAX_VEL = MAX_VEL, MAX_ACC = MAX_ACC;
    if(fabs(m_setPt - m_curPos) < MAX_VEL*MAX_VEL/MAX_ACC){ // for triangle motion profile
        m_speedDecreasePos = (m_setPt+m_curPos)/2;
    } else if (m_setPt > m_curPos)
        m_speedDecreasePos = m_setPt - MAX_VEL*MAX_VEL/(MAX_ACC*2);
    else 
        m_speedDecreasePos = m_setPt + MAX_VEL*MAX_VEL/(MAX_ACC*2);
}

//returns whether the wrist is at its setpoint
bool Wrist::AtSetpoint(){
    if (fabs(m_curPos - m_setPt) <= POS_TOLERANCE)
        return true;
    return false;
}

void Wrist::ResetPID(){
    m_totalErr = 0;
}

// for tuning, can test constant voltage on wrist or rollers 
// but need to pick which wrist or rollers in the code, since it cant be changed from shuffleboard
void Wrist::SetVoltage(){
    m_voltReq = std::clamp(m_voltReq, -MAX_VOLTS, MAX_VOLTS);
    m_wristMotor.SetVoltage(units::volt_t(m_voltReq));
}