#include "mbed.h"
#include "main.h"
#include "sx1272-hal.h"
#include "debug.h"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    868000000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm

#if USE_MODEM_LORA == 1

    #define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
    #define LORA_CODINGRATE                             1         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_FIX_LENGTH_PAYLOAD_ON                  false
    #define LORA_FHSS_ENABLED                           false  
    #define LORA_NB_SYMB_HOP                            4     
    #define LORA_IQ_INVERSION_ON                        false
    #define LORA_CRC_ENABLED                            true

#elif USE_MODEM_FSK == 1

    #define FSK_FDEV                                    25000     // Hz
    #define FSK_DATARATE                                19200     // bps
    #define FSK_BANDWIDTH                               50000     // Hz
    #define FSK_AFC_BANDWIDTH                           83333     // Hz
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_FIX_LENGTH_PAYLOAD_ON                   false
    #define FSK_CRC_ENABLED                             true

#else
    #error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     32        // Define the payload size here
//================ MODE SELECTION ============================== CHOOSE ONLY ONE
//#define RXMODE
#define TXMODE
//==============================================================================
#if( defined ( TARGET_KL25Z ) || defined ( TARGET_LPC11U6X ) )
DigitalOut led( LED2 );
#else
DigitalOut led( LED1 );
#endif


/*
 *  Global variables declarations
 */
 
typedef enum
{
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
}AppStates_t;

#ifdef RXMODE
Timer timer;
#endif

#ifdef TXMODE

InterruptIn user_button(USER_BUTTON);
#define DEFAULT_TX_DELAY 1000 // ms
#define DEFAULT_TX_SHIFT 4

#endif

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1272MB2xAS Radio( NULL );

const uint8_t Msg1[] = "LOR1";
const uint8_t Msg2[] = "LOR2";
const uint8_t Msg3[] = "LOR3";
#ifdef RXMODE
int32_t TimerValue=0;
int32_t TimeStampNew=0;
int32_t TimeStampOld=0;
#endif


uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

bool sendOn = false;

void button_isr(){
    sendOn = true;    
}


int main( void ) 
{
    uint8_t i;
    #ifdef TXMODE
        user_button.fall(&button_isr);
    #endif
    debug( "\n\n\r SX1272 Ping Pong Demo Application \n\n\r" );

    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents );

    // verify the connection with the board
    while( Radio.Read( REG_VERSION ) == 0x00  )
    {
        debug( "Radio could not be detected!\n\r", NULL );
        wait( 1 );
    }

    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1272MB2XAS ) ), "\n\r > Board Type: SX1272MB2xAS < \n\r" );

    Radio.SetChannel( RF_FREQUENCY ); 

#if USE_MODEM_LORA == 1

    debug_if( LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r" );
    debug_if( !LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r" );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, 2000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, true );

#elif USE_MODEM_FSK == 1

    debug("\n\n\r              > FSK Mode < \n\n\r" );
    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                         FSK_DATARATE, 0,
                         FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                         FSK_CRC_ENABLED, 0, 0, 0, 2000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                         0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                         0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, FSK_CRC_ENABLED,
                         0, 0, false, true );

#else

#error "Please define a modem in the compiler options."

#endif

    debug_if( DEBUG_MESSAGE, "Starting Ping-Pong loop\r\n" );

    led = 0;
    
#ifdef RXMODE
    Radio.Rx( RX_TIMEOUT_VALUE );
    timer.start();
#endif


    while( 1 )
    {
#ifdef RXMODE
        if(State ==  RX){   
                if( strncmp( ( const char* )Buffer, ( const char* )Msg1, 4 ) == 0 )
                {
                    TimeStampNew=timer.read_ms();
                    TimerValue=TimeStampNew-TimeStampOld;
                    TimeStampOld=TimeStampNew;
                    
                    led = !led;
                    debug( "I am LOR2\r\n" );
                    printf("Data received from...%s\r\n",Buffer);
                    printf("TimeStamp %d\n\r",TimeStampNew);
                    printf("Time %d\n\r",TimerValue);
                    Radio.Sleep( );
                    Buffer[0] = 'A';
                }//end if1
                wait_ms(10);    
                Radio.Rx( RX_TIMEOUT_VALUE );//SETTA LA RADIO IN MODALITA' RICEZIONE       
          }
#endif        
#ifdef TXMODE
    
    //    led = !led;
        if(sendOn == true){
        debug( "LOR1...\r\n" );
        SendMsg(Msg1, Buffer, BufferSize, DEFAULT_TX_DELAY, DEFAULT_TX_SHIFT);
        sendOn = false;
        }else{
        wait_ms(10);    
            }
#endif
    }
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    debug_if( DEBUG_MESSAGE, "> OnRxDone\n\r" );
    
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout( void )
{
    
    Radio.Sleep( );
    Buffer[BufferSize] = 0;
    State = RX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
    
}

void OnRxError( void )
{
    
    Radio.Sleep( );
    State = RX_ERROR;
    debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
    
}

void SendMsg(const uint8_t* Msg,uint8_t Buffer[], uint16_t BufferSize, int delay, int shift){
        strcpy( (char*)Buffer, (char*)Msg );
        int i;          
        
        for( i = shift; i < BufferSize; i++ )
        {
            Buffer[i] = i - shift;
        }     
        
        wait_ms( delay );
        Radio.Send( Buffer, BufferSize );
    }
    
uint8_t * Recieve(uint8_t* Buffer, uint8_t Msg, int shift){
    if( strncmp( ( const char* )Buffer, ( const char* )Msg, shift ) == 0 ){
                    
                    debug( "I am LOR2\r\n" );
                    printf("Data received from...%s\r\n",Buffer);
                    Radio.Sleep( );
                    Buffer[0] = 'A';
                }//end if1
                wait_ms(10);    
                Radio.Rx( RX_TIMEOUT_VALUE );
    }