/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2015 Semtech

Description: -

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#include "sx1272-hal.h"

#if defined ( TARGET_MOTE_L152RC )
/*
       PD_2=0  PD_2=1
op PaB  rfo     rfo
0  4.6  18.5    27.0
1  5.6  21.1    28.1
2  6.7  23.3    29.1
3  7.7  25.3    30.1
4  8.8  26.2    30.7
5  9.8  27.3    31.2
6  10.7 28.1    31.6
7  11.7 28.6    32.2
8  12.8 29.2    32.4
9  13.7 29.9    32.9
10 14.7 30.5    33.1
11 15.6 30.8    33.4
12 16.4 30.9    33.6
13 17.1 31.0    33.7
14 17.8 31.1    33.7
15 18.4 31.1    33.7
*/
//                           txpow:   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14  15  16  17  18  19
static const uint8_t PaBTable[20] = { 0, 0, 0, 0, 0, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15 };

//                           txpow:  20 21 22 23 24 25 26 27 28 29 30
static const uint8_t RfoTable[11] = { 1, 1, 1, 2, 2, 3, 4, 5, 6, 8, 9 };

#endif

const RadioRegisters_t SX1272MB2xAS::RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

SX1272MB2xAS::SX1272MB2xAS( RadioEvents_t *events,
                            PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                            PinName dio0, PinName dio1, PinName dio2, PinName dio3, PinName dio4, PinName dio5,
#if defined ( TARGET_MOTE_L152RC )
                            PinName rfSwitchCntr1, PinName rfSwitchCntr2 )
#elif defined ( TARGET_MTS_MDOT_F411RE )
                            PinName txctl, PinName rxctl )
#else
                            PinName antSwitch )
#endif
                            : SX1272( events, mosi, miso, sclk, nss, reset, dio0, dio1, dio2, dio3, dio4, dio5 ),
#if defined ( TARGET_MOTE_L152RC )
                            RfSwitchCntr1( rfSwitchCntr1 ),
                            RfSwitchCntr2( rfSwitchCntr2 ),
                            PwrAmpCntr( PD_2 )
#elif defined ( TARGET_MTS_MDOT_F411RE )
                            TxCtl ( txctl ),
                            RxCtl ( rxctl )
#else
                            AntSwitch( antSwitch ),
                        #if( defined ( TARGET_NUCLEO_L152RE ) ) || defined ( TARGET_NUCLEO_L476RG )
                            Fake( D8 )
                        #else
                            Fake( A3 )
                        #endif
#endif
{
    this->RadioEvents = events;

    Reset( );

    IoInit( );

    SetOpMode( RF_OPMODE_SLEEP );

    IoIrqInit( dioIrq );

    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = RF_IDLE ;
}

SX1272MB2xAS::SX1272MB2xAS( RadioEvents_t *events )
                        #if defined ( TARGET_NUCLEO_L152RE ) || defined ( TARGET_NUCLEO_L476RG )
                        :   SX1272( events, D11, D12, D13, D10, A0, D2, D3, D4, D5, A3, D9 ), // For NUCLEO L152RE dio4 is on port A3
                            AntSwitch( A4 ),
                            Fake( D8 )
                        #elif defined ( TARGET_MOTE_L152RC )
                        :   SX1272( events, PB_15, PB_14, PB_13, PB_12, PC_2, PC_6, PC_10, PC_11, PC_8, PC_9, PC_12 ),
                            RfSwitchCntr1( PC_4 ),
                            RfSwitchCntr2( PC_13 ),
                            PwrAmpCntr( PD_2 )
                        #elif defined ( TARGET_MTS_MDOT_F411RE )
                        :   SX1272( events, LORA_MOSI, LORA_MISO, LORA_SCK, LORA_NSS, LORA_RESET, LORA_DIO0, LORA_DIO1, LORA_DIO2, LORA_DIO3, LORA_DIO4, LORA_DIO5 ),
                            TxCtl( LORA_TXCTL ),
                            RxCtl( LORA_RXCTL )
                        #else
                        :   SX1272( events, D11, D12, D13, D10, A0, D2, D3, D4, D5, D8, D9 ),
                            AntSwitch( A4 ), 
                            Fake( A3 )
                        #endif
{
    this->RadioEvents = events;

    Reset( );

    boardConnected = UNKNOWN;

    DetectBoardType( );

    IoInit( );

    SetOpMode( RF_OPMODE_SLEEP );
    IoIrqInit( dioIrq );

    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = RF_IDLE ;
}

//-------------------------------------------------------------------------
//                      Board relative functions
//-------------------------------------------------------------------------
uint8_t SX1272MB2xAS::DetectBoardType( void )
{
    if( boardConnected == UNKNOWN )
    {
#if defined ( TARGET_MOTE_L152RC )
        boardConnected = NA_MOTE_72;
#elif defined ( TARGET_MTS_MDOT_F411RE )
        boardConnected = MDOT_F411RE;
#else
        this->AntSwitch.input( );
        wait_ms( 1 );
        if( this->AntSwitch == 1 )
        {
            boardConnected = SX1272MB1DCS;
        }
        else
        {
            boardConnected = SX1272MB2XAS;
        }
        this->AntSwitch.output( );
        wait_ms( 1 );
#endif
    }
    return ( boardConnected );
}

void SX1272MB2xAS::IoInit( void )
{
    AntSwInit( );
    SpiInit( );
}

void SX1272MB2xAS::RadioRegistersInit( )
{
    uint8_t i = 0;
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SetModem( RadioRegsInit[i].Modem );
        Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }    
}

void SX1272MB2xAS::SpiInit( void )
{
    nss = 1;    
    spi.format( 8,0 );   
    uint32_t frequencyToSet = 8000000;
    #if( defined ( TARGET_NUCLEO_L152RE ) || defined ( TARGET_MOTE_L152RC ) || defined ( TARGET_NUCLEO_L476RG ) ||  defined ( TARGET_LPC11U6X ) || defined ( TARGET_MTS_MDOT_F411RE ) )
        spi.frequency( frequencyToSet );
    #elif( defined ( TARGET_KL25Z ) ) //busclock frequency is halved -> double the spi frequency to compensate
        spi.frequency( frequencyToSet * 2 );
    #else
        #warning "Check the board's SPI frequency"
    #endif
    wait(0.1); 
}

void SX1272MB2xAS::IoIrqInit( DioIrqHandler *irqHandlers )
{
#if( defined ( TARGET_NUCLEO_L152RE ) || defined ( TARGET_MOTE_L152RC ) || defined ( TARGET_NUCLEO_L476RG ) || defined ( TARGET_NUCLEO_L476RG ) ||  defined ( TARGET_LPC11U6X ) )
    dio0.mode( PullDown );
    dio1.mode( PullDown );
    dio2.mode( PullDown );
    dio3.mode( PullDown );
    dio4.mode( PullDown );
#endif
    dio0.rise( mbed::callback( this, static_cast< TriggerMB2xAS > ( irqHandlers[0] ) ) );
    dio1.rise( mbed::callback( this, static_cast< TriggerMB2xAS > ( irqHandlers[1] ) ) );
    dio2.rise( mbed::callback( this, static_cast< TriggerMB2xAS > ( irqHandlers[2] ) ) );
    dio3.rise( mbed::callback( this, static_cast< TriggerMB2xAS > ( irqHandlers[3] ) ) );
    dio4.rise( mbed::callback( this, static_cast< TriggerMB2xAS > ( irqHandlers[4] ) ) );
}

void SX1272MB2xAS::IoDeInit( void )
{
    //nothing
}

void SX1272MB2xAS::SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = Read( REG_PACONFIG );
    paDac = Read( REG_PADAC );

#if defined ( TARGET_MOTE_L152RC )
    if( power > 19 )
    {
        paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_RFO;
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | RfoTable[power - 20];
    }
    else
    {
        paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_PABOOST;
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | PaBTable[power];
    }
#else
    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | GetPaSelect( this->settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
#endif
    Write( REG_PACONFIG, paConfig );
    Write( REG_PADAC, paDac );
}

uint8_t SX1272MB2xAS::GetPaSelect( uint32_t channel )
{
    if( boardConnected == SX1272MB1DCS || boardConnected == MDOT_F411RE )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1272MB2xAS::SetAntSwLowPower( bool status )
{
    if( isRadioActive != status )
    {
        isRadioActive = status;
    
        if( status == false )
        {
            AntSwInit( );
        }
        else
        {
            AntSwDeInit( );
        }
    }
}

void SX1272MB2xAS::AntSwInit( void )
{
#if defined ( TARGET_MOTE_L152RC )
    this->RfSwitchCntr1 = 0;
    this->RfSwitchCntr2 = 0;
    this->PwrAmpCntr = 0;
#elif defined ( TARGET_MTS_MDOT_F411RE )
    this->TxCtl = 0;
    this->RxCtl = 0;
#else
    this->AntSwitch = 0;
#endif
}

void SX1272MB2xAS::AntSwDeInit( void )
{
#if defined ( TARGET_MOTE_L152RC )
    this->RfSwitchCntr1 = 0;
    this->RfSwitchCntr2 = 0;
    this->PwrAmpCntr = 0;
#elif defined ( TARGET_MTS_MDOT_F411RE )
    this->TxCtl = 0;
    this->RxCtl = 0;
#else
    this->AntSwitch = 0;
#endif
}

void SX1272MB2xAS::SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
#if defined ( TARGET_MOTE_L152RC )
        if( ( Read( REG_PACONFIG ) & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
        {
            this->RfSwitchCntr1 = 1;
            this->RfSwitchCntr2 = 0;
        }
        else
        {
            this->RfSwitchCntr1 = 0;
            this->RfSwitchCntr2 = 1;
        }
#elif defined ( TARGET_MTS_MDOT_F411RE )
        /* SKY13350 */
        this->TxCtl = 1;
        this->RxCtl = 0;
#else
        this->AntSwitch = 1;
#endif
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
#if defined ( TARGET_MOTE_L152RC )
        this->RfSwitchCntr1 = 1;
        this->RfSwitchCntr2 = 1;
#elif defined ( TARGET_MTS_MDOT_F411RE )
        /* SKY13350 */
        this->TxCtl = 0;
        this->RxCtl = 1;
#else
        this->AntSwitch = 0;
#endif
        break;
    default:
#if defined ( TARGET_MOTE_L152RC )
        this->RfSwitchCntr1 = 0;
        this->RfSwitchCntr2 = 0;
        this->PwrAmpCntr = 0;
#elif defined ( TARGET_MTS_MDOT_F411RE )
        /* SKY13350 */
        this->TxCtl = 0;
        this->RxCtl = 0;
#else
        this->AntSwitch = 0;
#endif
        break;
    }
}

bool SX1272MB2xAS::CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

void SX1272MB2xAS::Reset( void )
{
    reset.output( );
    reset = 0;
    wait_ms( 1 );
    reset.input( );
    wait_ms( 6 );
}

void SX1272MB2xAS::Write( uint8_t addr, uint8_t data )
{
    Write( addr, &data, 1 );
}

uint8_t SX1272MB2xAS::Read( uint8_t addr )
{
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
}

void SX1272MB2xAS::Write( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    nss = 0;
    spi.write( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        spi.write( buffer[i] );
    }
    nss = 1;
}

void SX1272MB2xAS::Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    nss = 0;
    spi.write( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = spi.write( 0 );
    }
    nss = 1;
}

void SX1272MB2xAS::WriteFifo( uint8_t *buffer, uint8_t size )
{
    Write( 0, buffer, size );
}

void SX1272MB2xAS::ReadFifo( uint8_t *buffer, uint8_t size )
{
    Read( 0, buffer, size );
}
