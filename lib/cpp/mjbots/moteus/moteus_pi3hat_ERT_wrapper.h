#include "rtwtypes.h"
#include "moteus_protocol.h"
#include "pi3hat_moteus_interface.h"


// ------------------------
#ifdef __cplusplus
extern "C" {
#endif
// ------------------------

void Init_pi3hat(real64_T ctrl_period);
void setPosition(real64_T pos);
void readPosition(real64_T* out_pos);
void stopMotor();

// ------------------------
#ifdef __cplusplus
}
#endif
// ------------------------