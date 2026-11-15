#ifndef COMPETITION_PROFILE_H
#define COMPETITION_PROFILE_H

/* Build-time competition profile: PRELIM=1, FINAL=0.
 * Tests can override it on the compiler command line. No runtime code switch. */
#ifndef COMPETITION_PRELIMINARY
#define COMPETITION_PRELIMINARY 0
#endif

#if COMPETITION_PRELIMINARY
#define COMPETITION_VARIANT "PRELIM"
#define COMPETITION_ARM_BANK_SECTORS 8U
#define COMPETITION_DISC_BANK_SECTORS 14U
#else
#define COMPETITION_VARIANT "FINAL"
#define COMPETITION_ARM_BANK_SECTORS 12U
#define COMPETITION_DISC_BANK_SECTORS 10U
#endif

#endif
