#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_
// arm function
void armControl(void*ignore);
void setArmPos(int pos);
void armtiltSwitch();



// tilter function
void tiltControl(void*ignore);
void tiltSwitch();
void armState();
void scorestate();


#endif
