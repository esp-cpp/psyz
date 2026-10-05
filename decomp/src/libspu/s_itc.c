#include "libspu_private.h"
#include "libapi.h"

long SpuIsTransferCompleted(long immediateFlag) {
#ifdef __psyz
    /* psyz's transfers are synchronous (Psyz_SpuMemWrite copies in place), so
     * a transfer is complete the moment SpuWrite returns: both the wait and
     * the peek form answer yes. (The earlier version answered no to the peek,
     * which left drivers that poll it before the next write -- MGS's wave
     * loader -- believing a transfer was still running.) */
    (void)immediateFlag;
    return 1;
#endif
    s32 in_transfer;

    if (_spu_trans_mode == 1 || _spu_inTransfer == 1) {
        return 1;
    }
    in_transfer = TestEvent(_spu_EVdma);
    if (immediateFlag == 1) {
        if (in_transfer == 0) {
#ifndef __psyz // HACK the TestEvent must work
            do {
            } while (TestEvent(_spu_EVdma) == 0);
#endif
        }
        in_transfer = 1;
        _spu_inTransfer = in_transfer;
        return in_transfer;
    }
    if (in_transfer == 1) {
        _spu_inTransfer = in_transfer;
    }
    return in_transfer;
}
