#ifndef SENSOR_H
#define SENSOR_H

 /*  (GCLK_SOURCE / GCLK_DIVIDE) / (PRESCALER + REGISTER COUNTS) = OVERFLOW FREQUENCE OF TCX
 *  (48Mhz / GCLK_DIVID) / PRESCALER + 2^16) = 2^16µS means
 *  Currenlty setted up to 1MHZ which equals count stamps of 1µS */
void initCounter()
{
    // divides the source frequence of the GLCK with the provided divide value
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |         
        GCLK_GENDIV_ID(4);           
    while (GCLK->STATUS.bit.SYNCBUSY);              

    // 48MHz source for the GCLK
    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          
        GCLK_GENCTRL_GENEN |         
        GCLK_GENCTRL_SRC_DFLL48M |   
        GCLK_GENCTRL_ID(4);          
    while (GCLK->STATUS.bit.SYNCBUSY);           

    // passes GEN_GCLK4 to the counters TC4 and TC5
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |             
        GCLK_CLKCTRL_GEN_GCLK4 |                                
        GCLK_CLKCTRL_ID_TC4_TC5;                            
    while (GCLK->STATUS.bit.SYNCBUSY);                  
    REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT16;             
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);           

    REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV16|                 
        TC_CTRLA_ENABLE;                                    
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);          
}

volatile static uint16_t startT;
volatile static uint16_t stopT;
volatile static uint16_t duration;
volatile static uint16_t duration_prev;

typedef struct Sweep{
    uint16_t        magicNumber;
    uint8_t         sensorID;
    uint8_t         lighthouse;
    uint8_t         rotor;
    uint16_t        sweepDuration; 
}Sweep; 


typedef struct _FIFO128sweep{
    uint8_t     mRead;
    uint8_t     mWrite;
    Sweep *      mBuffer[128]; 
}FIFO128sweep; 

typedef struct _FIFO128t{
    uint8_t     mRead;
    uint8_t     mWrite;
    uint16_t    mBuffer[128]; 
}FIFO128t; 

volatile static bool vertical;
volatile static bool sweep_active;
volatile static bool synced;
volatile static bool BaseStation;
volatile static bool lighthouse;
volatile static uint16_t sweep_start;
volatile static uint16_t sweep_duration;
volatile static uint16_t active_sweep_a;
volatile static uint16_t active_sweep_b;
volatile static FIFO128sweep sweepFIFO;

//FIFO Operations are implemented as Preprocessor Functions since the FIFO is used in several IRQs an things need to get a bit faster
#define FIFO_init(fifo)                 { fifo.mRead = 0; fifo.mWrite = 0;}

#define FIFO_available(fifo)            ( fifo.mRead != fifo.mWrite )

#define FIFO_read(fifo, size)           (                                           \
        (FIFO_available(fifo)) ?                                                    \
        fifo.mBuffer[(fifo.mRead = ((fifo.mRead + 1) & (size -1)))] : 0             \
        ) 

#define FIFO_write(fifo, data, size)    {                                           \
    uint8_t temp = ((fifo.mWrite +1 )& (size -1));                                  \
    if(temp != fifo.mRead) {                                                        \
        fifo.mBuffer[temp] = data;                                                  \
        fifo.mWrite = temp;                                                         \
    }                                                                               \
}

#define FIFO128_read(fifo)              FIFO_read(fifo, 128)
#define FIFO128_write(fifo, data)       FIFO_write(fifo, data, 128)

void rising_IRQ_S1(void)
{
    startT = (uint16_t) (TC4->COUNT16.COUNT.reg); 
}

void falling_IRQ_S1(void)
{
    stopT = (uint16_t) (TC4->COUNT16.COUNT.reg); 
    duration = (stopT - startT); 

    // sync pulse detected! Get duration and start sweep-counting measurment 
    if( duration > 50 )
    {
        // check if the sync pulse signals skip or not
        if((60 < duration && 70 > duration)
                || ( 81 < duration && 90 > duration))
        {
            vertical = true; 
            synced = true; 
            sweep_active = true; 
            sweep_start = startT; 

        }else if(synced == false){
        }else if(( 71 < duration && 80 > duration)
                || ( 91 < duration && 100 > duration))
        {
            sweep_start = startT; 
            vertical = false; 
            sweep_active = true; 
        }
    }
    // laser sweep detected! Complete sweep-counting measurement 
    else if(true == sweep_active && duration < 50)
    {
        sweep_active = false; 
        sweep_duration = startT - sweep_start ; 
        if(BaseStation == false){
            active_sweep_a = startT; 
            BaseStation = true; 
        }else{
            active_sweep_b = startT; 
            uint16_t baseStationGap= ((active_sweep_b - active_sweep_a)); 
            if(baseStationGap > 6000)
            {
                lighthouse = 1;
            }else{
                lighthouse = 0;
            }

            BaseStation = false; 
        }   

        Sweep * sweep = static_cast<Sweep*>(malloc( sizeof(Sweep))); 
        if(sweep != NULL){
            uint8_t sensorID = 0;
            sweep->lighthouse = lighthouse;
            sweep->rotor = vertical;
            sweep->sensorID = 0;
            sweep->sweepDuration   = sweep_duration; 
            sweep->magicNumber     = 0xBEEF;
        }
       
        FIFO128_write(sweepFIFO, sweep);  
    }
}


#endif
