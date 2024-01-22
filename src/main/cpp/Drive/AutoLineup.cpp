#include "Drive/AutoLineup.h"
#include <cmath>

void AutoLineup::SetTarget(vec::Vector2D newTarget){
    double xdiff = newTarget.x() - m_odom.GetPos().x(), ydiff= newTarget.y() - m_odom.GetPos().y();
    double dist = hypot(xdiff, ydiff);
    double ang = atan2(ydiff, xdiff);

    createProfile(m_pos, m_odom.GetVel().magn(), dist);
    createProfile(m_ang, m_odom.GetAngVel(), ang, m_odom.GetAng());
}

AutoLineup::AutoLineup(Odometry &odom):m_odom{odom}{
    m_ang.MAX_ACC = 0.0;
    m_ang.MAX_VEL = 0.0;
    m_pos.MAX_ACC = 0.0;
    m_pos.MAX_VEL = 0.0;
}

//todo change targ acceleration to be "negative" if vi is "above" max vel
void AutoLineup::createProfile(profileInfo p, double vi, double targPos, double curPos = 0){
    // if (vi < MAX_VEl) must keep in mind sign tho
    double deltaPos = targPos-curPos;
    double p1 = (square(p.MAX_VEL) - square(vi))/(2*p.MAX_ACC);
    double p3 = square(p.MAX_VEL)/(2*p.MAX_ACC);
    if (p1+p3 > deltaPos){
        p.speedDecreasePos = deltaPos- square(deltaPos*p.MAX_ACC+square(vi)/2)/p.MAX_ACC;
    } else {
        p.speedDecreasePos = (deltaPos -p1 -p3)/p.MAX_VEL;
    }
    p.targPose = {curPos, vi, p.MAX_ACC};
    p.setPt = targPos;
}

//todo: everything lolz
void AutoLineup::UpdateProfile(profileInfo p){
    Pose1D t = p.targPose;
    // t.pos += t.vel*0.02;
    // t.vel = std::clamp(t.vel+t.acc*0.02, -p.MAX_VEL, p.MAX_VEL);
    // if (t.pos >= p.speedDecreasePos){ // need to sign check this
    //     t.acc = -p.MAX_ACC;
    // }

    double newP = t.pos, newV = t.vel, newA = t.acc;


    newP += t.vel * 0.02;
    newV += t.acc * 0.02;

    if (p.speedDecreasePos < p.setPt){ // if trapezoid is pos
        if (newP > p.speedDecreasePos) // if after turn pt
            newV = std::max(0.0, p.targetVel - IntakeConstants::WRIST_MAX_ACC * 0.02);
        else 
            newV = std::min(IntakeConstants::WRIST_MAX_VEL, p.targetVel + IntakeConstants::WRIST_MAX_ACC * 0.02);
    } else {
        if (newP > m_speedDecreasePos) // if before the turn pt
            newV = std::max(-IntakeConstants::WRIST_MAX_VEL, m_targetVel - IntakeConstants::WRIST_MAX_ACC * 0.02);
        else 
            newV = std::min(0.0, m_targetVel + IntakeConstants::WRIST_MAX_ACC * 0.02);
    }

    if (newV-m_targetVel == 0) newA = 0;
    else if (newV > m_targetVel) newA = IntakeConstants::WRIST_MAX_ACC;
    else newA = -IntakeConstants::WRIST_MAX_ACC;

    m_targetPos = newP;
    m_targetVel = newV;
    m_targetAcc = newA;

}

double square(double d){
    return d*d;
}

void AutoLineup::Periodic(){
    UpdateProfile(m_ang);
    UpdateProfile(m_pos);
}

vec::Vector2D AutoLineup::GetVel() const {
    double vmag =m_pos.targPose.vel;
    return {std::cos(m_ang.setPt)*vmag, std::sin(m_ang.setPt)*vmag};
}

double AutoLineup::GetAngVel() const {
    return m_ang.targPose.vel;
}

double AutoLineup::GetAng() const {
    return m_ang.targPose.pos;
}