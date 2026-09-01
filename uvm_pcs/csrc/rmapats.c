// file = 0; split type = patterns; threshold = 100000; total count = 0.
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include "rmapats.h"

void  schedNewEvent (struct dummyq_struct * I1417, EBLK  * I1412, U  I623);
void  schedNewEvent (struct dummyq_struct * I1417, EBLK  * I1412, U  I623)
{
    U  I1683;
    U  I1684;
    U  I1685;
    struct futq * I1686;
    struct dummyq_struct * pQ = I1417;
    I1683 = ((U )vcs_clocks) + I623;
    I1685 = I1683 & ((1 << fHashTableSize) - 1);
    I1412->I668 = (EBLK  *)(-1);
    I1412->I669 = I1683;
    if (0 && rmaProfEvtProp) {
        vcs_simpSetEBlkEvtID(I1412);
    }
    if (I1683 < (U )vcs_clocks) {
        I1684 = ((U  *)&vcs_clocks)[1];
        sched_millenium(pQ, I1412, I1684 + 1, I1683);
    }
    else if ((peblkFutQ1Head != ((void *)0)) && (I623 == 1)) {
        I1412->I671 = (struct eblk *)peblkFutQ1Tail;
        peblkFutQ1Tail->I668 = I1412;
        peblkFutQ1Tail = I1412;
    }
    else if ((I1686 = pQ->I1320[I1685].I691)) {
        I1412->I671 = (struct eblk *)I1686->I689;
        I1686->I689->I668 = (RP )I1412;
        I1686->I689 = (RmaEblk  *)I1412;
    }
    else {
        sched_hsopt(pQ, I1412, I1683);
    }
}
#ifdef __cplusplus
extern "C" {
#endif
void SinitHsimPats(void);
#ifdef __cplusplus
}
#endif
