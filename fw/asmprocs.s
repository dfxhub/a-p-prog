    module asmprocs
    #include "iostm8s003f3.h"
    public _delay_us_asm
    section .near_func.text:code
    ;
    ;
    ;
_delay_us_asm:
    ;    sllw X              ; 2 cy
    ;    sllw X              ; 2 cy
us_loop:
    decw X              ; 1 cy
    jreq us_fin         ; 1 or 2 when jump cy
    jrne us_loop        ; 1 or 2 when jymp cy
us_fin:
    ret                 ; 4 cy
    ;
    ;
    ;
    end