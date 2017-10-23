require '_h2ph_pre.ph';

no warnings qw(redefine misc);

unless(defined(&_BITS_TIMEX_H)) {
    eval 'sub _BITS_TIMEX_H () {1;}' unless defined(&_BITS_TIMEX_H);
    require 'bits/types.ph';
    eval 'sub ADJ_OFFSET () {0x1;}' unless defined(&ADJ_OFFSET);
    eval 'sub ADJ_FREQUENCY () {0x2;}' unless defined(&ADJ_FREQUENCY);
    eval 'sub ADJ_MAXERROR () {0x4;}' unless defined(&ADJ_MAXERROR);
    eval 'sub ADJ_ESTERROR () {0x8;}' unless defined(&ADJ_ESTERROR);
    eval 'sub ADJ_STATUS () {0x10;}' unless defined(&ADJ_STATUS);
    eval 'sub ADJ_TIMECONST () {0x20;}' unless defined(&ADJ_TIMECONST);
    eval 'sub ADJ_TAI () {0x80;}' unless defined(&ADJ_TAI);
    eval 'sub ADJ_MICRO () {0x1000;}' unless defined(&ADJ_MICRO);
    eval 'sub ADJ_NANO () {0x2000;}' unless defined(&ADJ_NANO);
    eval 'sub ADJ_TICK () {0x4000;}' unless defined(&ADJ_TICK);
    eval 'sub ADJ_OFFSET_SINGLESHOT () {0x8001;}' unless defined(&ADJ_OFFSET_SINGLESHOT);
    eval 'sub ADJ_OFFSET_SS_READ () {0xa001;}' unless defined(&ADJ_OFFSET_SS_READ);
    eval 'sub MOD_OFFSET () { &ADJ_OFFSET;}' unless defined(&MOD_OFFSET);
    eval 'sub MOD_FREQUENCY () { &ADJ_FREQUENCY;}' unless defined(&MOD_FREQUENCY);
    eval 'sub MOD_MAXERROR () { &ADJ_MAXERROR;}' unless defined(&MOD_MAXERROR);
    eval 'sub MOD_ESTERROR () { &ADJ_ESTERROR;}' unless defined(&MOD_ESTERROR);
    eval 'sub MOD_STATUS () { &ADJ_STATUS;}' unless defined(&MOD_STATUS);
    eval 'sub MOD_TIMECONST () { &ADJ_TIMECONST;}' unless defined(&MOD_TIMECONST);
    eval 'sub MOD_CLKB () { &ADJ_TICK;}' unless defined(&MOD_CLKB);
    eval 'sub MOD_CLKA () { &ADJ_OFFSET_SINGLESHOT;}' unless defined(&MOD_CLKA);
    eval 'sub MOD_TAI () { &ADJ_TAI;}' unless defined(&MOD_TAI);
    eval 'sub MOD_MICRO () { &ADJ_MICRO;}' unless defined(&MOD_MICRO);
    eval 'sub MOD_NANO () { &ADJ_NANO;}' unless defined(&MOD_NANO);
    eval 'sub STA_PLL () {0x1;}' unless defined(&STA_PLL);
    eval 'sub STA_PPSFREQ () {0x2;}' unless defined(&STA_PPSFREQ);
    eval 'sub STA_PPSTIME () {0x4;}' unless defined(&STA_PPSTIME);
    eval 'sub STA_FLL () {0x8;}' unless defined(&STA_FLL);
    eval 'sub STA_INS () {0x10;}' unless defined(&STA_INS);
    eval 'sub STA_DEL () {0x20;}' unless defined(&STA_DEL);
    eval 'sub STA_UNSYNC () {0x40;}' unless defined(&STA_UNSYNC);
    eval 'sub STA_FREQHOLD () {0x80;}' unless defined(&STA_FREQHOLD);
    eval 'sub STA_PPSSIGNAL () {0x100;}' unless defined(&STA_PPSSIGNAL);
    eval 'sub STA_PPSJITTER () {0x200;}' unless defined(&STA_PPSJITTER);
    eval 'sub STA_PPSWANDER () {0x400;}' unless defined(&STA_PPSWANDER);
    eval 'sub STA_PPSERROR () {0x800;}' unless defined(&STA_PPSERROR);
    eval 'sub STA_CLOCKERR () {0x1000;}' unless defined(&STA_CLOCKERR);
    eval 'sub STA_NANO () {0x2000;}' unless defined(&STA_NANO);
    eval 'sub STA_MODE () {0x4000;}' unless defined(&STA_MODE);
    eval 'sub STA_CLK () {0x8000;}' unless defined(&STA_CLK);
    eval 'sub STA_RONLY () {( &STA_PPSSIGNAL |  &STA_PPSJITTER |  &STA_PPSWANDER |  &STA_PPSERROR |  &STA_CLOCKERR |  &STA_NANO |  &STA_MODE |  &STA_CLK);}' unless defined(&STA_RONLY);
}
1;
