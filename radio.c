/*******************************************************************************************************
 * @file     radio.c
 * @brief    nRF52832 radio driver
 * @version  1.0
 * @date     2017-6-8
 * @author   Nikko
 *******************************************************************************************************
 *******************************************************************************************************
 * History:
 *------------------------------------------------------------------------------------------------------
 *|  Date       | Version   |   Description
 *| 2017-6-8    | V1.0.0    |  First release
 *******************************************************************************************************/

/* Includes ------------------------------------------------------------------------------------------ */
#include "nrf_delay.h"

static const uint32_t	_PLD_MAXSIZE_ = 64UL;

/* These are set to zero as Shockburst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE      (3UL)  /**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE      (0UL)  /**< Packet S0 field size in bits. */
#define PACKET_LENGTH_FIELD_SIZE  (7UL)  /**< Packet length field size in bits. */


#define PACKET_BASE_ADDRESS_LENGTH  (4UL)	//!< Packet base address length field size in bytes
#define PACKET_STATIC_LENGTH        (0UL)	//!< Packet static length in bytes
#define PACKET_PAYLOAD_MAXSIZE      (_PLD_MAXSIZE_+2)	//!< Packet payload maximum size in bytes


// Packet configuration
#define RADIO_PKT_CNF 	(RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |	\
                       (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |		\
                       (PACKET_BASE_ADDRESS_LENGTH   << RADIO_PCNF1_BALEN_Pos)   |		\
                       (PACKET_STATIC_LENGTH         << RADIO_PCNF1_STATLEN_Pos) |		\
                       (PACKET_PAYLOAD_MAXSIZE       << RADIO_PCNF1_MAXLEN_Pos)

/**
 * @brief Function for swapping/mirroring bits in a byte.
 * 
 *@verbatim
 * output_bit_7 = input_bit_0
 * output_bit_6 = input_bit_1
 *           :
 * output_bit_0 = input_bit_7
 *@endverbatim
 *
 * @param[in] inp is the input byte to be swapped.
 *
 * @return
 * Returns the swapped/mirrored input byte.
 */
static uint32_t swap_bits(uint32_t inp);

/**
 * @brief Function for swapping bits in a 32 bit word for each byte individually.
 * 
 * The bits are swapped as follows:
 * @verbatim
 * output[31:24] = input[24:31] 
 * output[23:16] = input[16:23]
 * output[15:8]  = input[8:15]
 * output[7:0]   = input[0:7]
 * @endverbatim
 * @param[in] input is the input word to be swapped.
 *
 * @return
 * Returns the swapped input byte.
 */
static uint32_t bytewise_bitswap(uint32_t inp);

static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;
    
    inp = (inp & 0x000000FFUL);
    
    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);     
    }
    
    return retval;    
}


static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
}

/** 
 * @brief Function for configuring the radio to operate in Shockburst compatible mode.
 * 
 * To configure the application running on nRF24L series devices:
 *
 * @verbatim
 * uint8_t tx_address[5] = { 0xA7, 0x7E, 0x3C, 0xC1, 0xC0 };
 * hal_nrf_set_rf_channel(7);
 * hal_nrf_set_address_width(HAL_NRF_AW_5BYTES); 
 * hal_nrf_set_address(HAL_NRF_TX, tx_address);
 * hal_nrf_set_address(HAL_NRF_PIPE0, tx_address); 
 * hal_nrf_open_pipe(0, false);
 * hal_nrf_set_datarate(HAL_NRF_2MBPS);
 * hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);
 * hal_nrf_setup_dynamic_payload(0xFF);
 * hal_nrf_enable_dynamic_payload(false);
 * @endverbatim
 *
 * When transmitting packets with hal_nrf_write_tx_payload(const uint8_t *tx_pload, uint8_t length),
 * match the length with PACKET_STATIC_LENGTH.
 * hal_nrf_write_tx_payload(payload, PACKET_STATIC_LENGTH);
 * 
*/
void Radio_Init(void)
{
   // radio powered on
    NRF_RADIO->POWER = 1;  
    
    // radio disable
    NRF_RADIO->EVENTS_DISABLED = 0U;
    NRF_RADIO->TASKS_DISABLE   = 1U;    
    while(NRF_RADIO->EVENTS_DISABLED == 0U){};
	
    // Radio config
    NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->FREQUENCY = 7UL;  // Frequency bin 7, 2407MHz
    NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);

    // Radio address config
    NRF_RADIO->PREFIX0 = 
        ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC0) << 0); // Prefix byte of address 0 converted to nRF24L series format
  
    NRF_RADIO->PREFIX1 = 
        ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
	    | ((uint32_t)swap_bits(0xC5) << 8)  // Prefix byte of address 5 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

    NRF_RADIO->BASE0 = bytewise_bitswap(0xA77E3CC1UL);  // Base address for prefix 0 converted to nRF24L series format
    NRF_RADIO->BASE1 = bytewise_bitswap(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format
  
    NRF_RADIO->TXADDRESS   = 0x00UL;  // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = 0x01UL;  // Enable device address 0 to use to select which addresses to receive

    // Packet configuration
    NRF_RADIO->PCNF0 = (PACKET_S1_FIELD_SIZE     << RADIO_PCNF0_S1LEN_Pos) |
                       (PACKET_S0_FIELD_SIZE     << RADIO_PCNF0_S0LEN_Pos) |
                       (PACKET_LENGTH_FIELD_SIZE << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // Packet configuration
    NRF_RADIO->PCNF1 = RADIO_PKT_CNF;
	
    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value      
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16+x^12^x^5+1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8+x^2^x^1+1
    }
	
//	NRF_RADIO->SHORTS = 
//		  ( RADIO_SHORTS_DISABLED_RSSISTOP_Enabled	<< RADIO_SHORTS_DISABLED_RSSISTOP_Pos )
//		| ( RADIO_SHORTS_ADDRESS_RSSISTART_Enabled	<< RADIO_SHORTS_ADDRESS_RSSISTART_Pos )
//		| ( RADIO_SHORTS_READY_START_Enabled		<< RADIO_SHORTS_READY_START_Pos ) ;
////		| ( RADIO_SHORTS_END_DISABLE_Enabled		<< RADIO_SHORTS_END_DISABLE_Pos) ;
	
	// Fast ramp-up (tRXEN,FAST) mode
	NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
	
//	NRF_RADIO->INTENSET = (RADIO_INTENSET_END_Enabled << RADIO_INTENSET_END_Pos);
//	
//	NVIC_EnableIRQ(RADIO_IRQn);
}


static void Radio_Disable(void)
{
	NRF_RADIO->EVENTS_DISABLED = 0U; 
    NRF_RADIO->TASKS_DISABLE   = 1U;
    while(NRF_RADIO->EVENTS_DISABLED == 0U) {};
}


int32_t Radio_Send(uint8_t *buf, int32_t len)
{
	static uint8_t PID = 0;
	int32_t ARC = 3, ARD = 500;
	uint8_t Packet[_PLD_MAXSIZE_ + 2];
	
	int32_t toSnd = len, Snd;
	int32_t i;
	
	if (toSnd > 64)
		toSnd = 64;
	// Tx buffer format ...
	Packet[0] = toSnd;
	Packet[1] = (PID++)<<1;
	memcpy(Packet+2, buf, len);
	NRF_RADIO->PACKETPTR = (uint32_t)Packet;
	NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos);
	
	// Try to send buffer, end when ARC = 0
	while (ARC--)
	{
		NRF_RADIO->EVENTS_READY = 0;
		NRF_RADIO->EVENTS_END = 0;
		NRF_RADIO->TASKS_TXEN = 1;
		while (NRF_RADIO->EVENTS_END == 0);	// Wait for TX done
		NRF_RADIO->EVENTS_END = 0;
		Radio_Disable();
		NRF_RADIO->EVENTS_READY = 0;
		NRF_RADIO->TASKS_RXEN = 1;
		
		// Start listenning packet
		i = 0;
		while (1)
		{
			if (++i == ARD)
				break;
			nrf_delay_us(1);
			// Start searching for packets
			if (NRF_RADIO->EVENTS_END == 0)
				continue;
			NRF_RADIO->EVENTS_END = 0;
			// CRC OK?
			if (NRF_RADIO->CRCSTATUS == 0)
			{// CRC error, restart RX
				NRF_RADIO->TASKS_START = 1;
				continue;
			}
			// Check the packet is a ACK packet
			if (Packet[1]&0x01)
			{
				NRF_RADIO->TASKS_START = 1;
				continue;
			}
			// now, received ACK
			Snd = toSnd;
//			// Do a ACK have payload ?
//			if (Packet[0] != 0)
//			{// Read the ACK payload
//			}
			break;
		}	// end of while (1) ... Listenning packet
	}// end of while (ARC--)	... Try to re-send 
	
	Radio_Disable();
	
	return Snd;
}

int32_t Radio_ListenLoopback(uint8_t *buf, int32_t bufSize, int32_t *Rssi, 
        void (*fpRcv_cbFunc)(uint8_t*, int32_t))
{
	static volatile uint8_t	lastPID = 0;
	static volatile uint32_t lastCRC = 0;
	static volatile uint8_t Packet[_PLD_MAXSIZE_ + 2];
	volatile uint8_t rxPID;
	volatile uint32_t rxCRC;
	
	int32_t Rcv = 0;
	
	if ((NRF_RADIO->STATE != RADIO_STATE_STATE_RxIdle) && (NRF_RADIO->STATE != RADIO_STATE_STATE_Rx))
	{
		Radio_Disable();
		NRF_RADIO->PACKETPTR = (uint32_t)Packet;
		NRF_RADIO->SHORTS = 
				  ( RADIO_SHORTS_DISABLED_RSSISTOP_Enabled	<< RADIO_SHORTS_DISABLED_RSSISTOP_Pos )
				| ( RADIO_SHORTS_ADDRESS_RSSISTART_Enabled	<< RADIO_SHORTS_ADDRESS_RSSISTART_Pos )
				| ( RADIO_SHORTS_READY_START_Enabled		<< RADIO_SHORTS_READY_START_Pos ) ;
		
		NRF_RADIO->EVENTS_READY = 0;
		NRF_RADIO->EVENTS_END = 0;
		NRF_RADIO->TASKS_START = 1;
		// Wait for Radio enter to RxIdle state
//		nrf_delay_us(140);	// Time between the RXEN task and READY event in default mode
		nrf_delay_us(40);	// Time between the RXEN task and READY event in fast mode
	}
	
	// Start searching for packets
	if (NRF_RADIO->EVENTS_END == 0)
		return Rcv;
	NRF_RADIO->EVENTS_END = 0;
	if (NRF_RADIO->CRCSTATUS == 0)
	{
		NRF_RADIO->TASKS_START = 1;
		return Rcv;
	}
	// Rcvd a packet
	rxPID = (Packet[1]>>1)&0x03;
	rxCRC = NRF_RADIO->RXCRC;
	// Is the received packet a new packet ?
	if ((rxPID == lastPID) && (rxCRC == lastCRC))
	{
		// do nothing, discard packet as a copy
	}
	else
	{// New packet is valid
		lastCRC = rxCRC;
		lastPID = rxPID;
		
		Rcv = Packet[0];
		if (Rcv > bufSize)
			Rcv = bufSize;
		memcpy(buf, (void*)(Packet+2), Rcv);
		*Rssi = ~NRF_RADIO->RSSISAMPLE + 1;	// RSSI Accuracy Valid range -90 to -20 dBm
		
		// Respone ACK
		Packet[0] = 0;
		Packet[1] = 0;
		Radio_Disable();
		NRF_RADIO->EVENTS_READY = 0;
		NRF_RADIO->EVENTS_END = 0;
		NRF_RADIO->TASKS_TXEN = 1;
		while (NRF_RADIO->EVENTS_END == 0);
		NRF_RADIO->EVENTS_END = 0;
		
		// turn back to the Rx listening State
		Radio_Disable();
		NRF_RADIO->TASKS_RXEN = 1;
		
		if (fpRcv_cbFunc)
			fpRcv_cbFunc(buf, bufSize);
	}
	
	return Rcv;
}



/*********************************************************************************************************
    End Of File
*********************************************************************************************************/
