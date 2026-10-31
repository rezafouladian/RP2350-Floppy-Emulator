; IIgs helpers for a2bench, callable from cc65 C (ProDOS 8, emulation mode).
;
;   unsigned char iigs(void)        0 if not a IIgs, else 1 + the ROM version (IDRoutine's Y)
;   void __fastcall__ rtc_read(unsigned char *hms)
;                                   the battery clock: hms[0] seconds, [1] minutes, [2] hours
;                                   (Misc Tools' ReadTimeHex, in full native mode)
;   unsigned char vbl(void)         $80 while the beam is in vertical blanking (RDVBLBAR)
;   unsigned char fast(void)        $80 if the IIgs runs at 2.8 MHz (the speed register)
;   void __fastcall__ cout(char c)  the monitor's COUT (scrolls; through the 80-column firmware
;                                   once on)
;   void home(void)                 the monitor's HOME
;   void col80(void)                the 80-column firmware on (PR#3)

        .setcpu "65816"
        .export _iigs, _rtc_read, _vbl, _fast, _cout, _home, _col80
        .importzp ptr1

        .segment "CODE"

; The monitor ROM, which cc65's startup may have switched the language card's RAM over: rom_in
; switches the ROM in (keeping A), rom_out the RAM back as it was (keeping A; cc65 runs from
; bank 2).
rom_in: pha
        lda     $C012           ; RDLCRAM: language card RAM read-enabled
        sta     lcram
        lda     $C011           ; RDLCBNK2
        sta     lcbank
        bit     $C082           ; read the ROM
        pla
        rts

rom_out:
        bit     lcram
        bpl     @rom
        bit     lcbank
        bmi     @b2
        bit     $C088           ; RAM bank 1, read
        rts
@b2:    bit     $C080           ; RAM bank 2, read
        rts
@rom:   bit     $C082
        rts

_iigs:  jsr     rom_in
        sec
        jsr     $FE1F           ; IDRoutine: an RTS before the IIgs, so carry stays set
        lda     #0
        bcs     @done
        tya                     ; the ROM version
        clc
        adc     #1
@done:  jsr     rom_out
        ldx     #0
        rts

_cout:  ora     #$80
        jsr     rom_in
        jsr     $FDED           ; COUT
        jmp     rom_out

_home:  jsr     rom_in
        jsr     $FC58           ; HOME
        jmp     rom_out

_col80: jsr     rom_in
        jsr     $C300           ; the 80-column firmware's entry, as PR#3
        jmp     rom_out

_rtc_read:
        sta     ptr1
        stx     ptr1+1
        php
        clc
        xce                     ; native
        rep     #$30
        .a16
        .i16
        pha                     ; room for ReadTimeHex's 8 bytes
        pha
        pha
        pha
        ldx     #$0D03          ; ReadTimeHex (Misc Tools)
        jsl     $E10000
        pla                     ; second, minute
        sta     secmin
        pla                     ; hour, year
        sta     houryr
        pla                     ; day, month
        pla                     ; null, weekday
        sep     #$30
        .a8
        .i8
        sec
        xce                     ; back to emulation
        plp
        ldy     #0
        lda     secmin
        sta     (ptr1),y
        iny
        lda     secmin+1
        sta     (ptr1),y
        iny
        lda     houryr
        sta     (ptr1),y
        rts

_vbl:   lda     $C019
        and     #$80
        ldx     #0
        rts

_fast:  lda     $C036
        and     #$80
        ldx     #0
        rts

        .segment "BSS"
secmin: .res    2
houryr: .res    2
lcram:  .res    1
lcbank: .res    1
