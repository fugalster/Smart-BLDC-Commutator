//#include <EEPROM.h> // eeprom not used currently
// TODO: have configurable settings commanded by master over SPI, stored in eeprom



volatile uint8_t phase = 0; // motor firing phase
volatile long position = 0; // current position
volatile bool sample_and_hold_pin = false; // flag to tell loop to sample current position
volatile byte position_buffer[] = {0, 0, 0, 0}; // buffer for transfering single bytes of position over spi



void setup() {

    // configure motor control outputs
    DDRC |= 0x3F;  // A0-A5 as outputs
    // equivalent of:
    //   pinMode(A0, OUTPUT);
    //   pinMode(A1, OUTPUT);
    //   pinMode(A2, OUTPUT);
    //   pinMode(A3, OUTPUT);
    //   pinMode(A4, OUTPUT);
    //   pinMode(A5, OUTPUT);

    // configure uC control pins
    DDRD &= !0xEF; // 0-3, 5-7 as inputs (will break serial communication and require ISP programmer to undo)
    DDRD |= 0x10;  // 4 as output, used for software pin change interrupt
    PORTD |= 0xE0; // turn on pullup resistors
    // equivalent of:
    //   pinMode(0, INPUT);  // sample and hold
    //   pinMode(1, INPUT);  // motor direction
    //   pinMode(2, INPUT);  // encoder channel B
    //   pinMode(3, INPUT);  // encoder channel A
    //   pinMode(4, OUTPUT); // used for software initiated pin change interrupt
    //   pinMode(5, INPUT);  // hall sensor 3
    //   pinMode(6, INPUT);  // hall sensor 2
    //   pinMode(7, INPUT);  // hall sensor 1

    // configure misc pins
    pinMode(8, OUTPUT); // status LED
    pinMode(9, INPUT);  // pwm pin, master controls this

    // arduino doesn't have built in libraries for slave mode SPI so we'll do it all manually
    // turn on SPI in slave mode
    pinMode(MISO, OUTPUT);
    pinMode(SS, INPUT); // /SS pin
    SPCR |= 0b00;       // SPI speed f_osc/4 (4MHz with 16MHz chip)
    SPCR &= ~_BV(DORD); // MSB first
    SPCR &= ~_BV(MSTR); // slave mode
    SPCR &= ~_BV(CPOL); // clock low when idle
    SPCR &= ~_BV(CPHA); // sample on clock rising edge
    SPCR |=  _BV(SPE);  // enable SPI
    SPCR |=  _BV(SPIE); // enable SPI interrupts

    // initialize interrupts
    initInterrupt();

    // setup phase to correct motor position by triggering pin change interrupt a few times
    // fills in hall variable in pin change ISR 
    delay(1);
    digitalWrite(4, LOW);
    delay(1);
    digitalWrite(4, HIGH);
    delay(1);
    digitalWrite(4, LOW);
	
	// flash LED thrice on reset
	for (int i=0; i<6; i++) {
	    PINB |= _BV(0);
		delay(80);
	}
}



void loop() {

    static long sample_position = 0;
    static bool sample_and_hold_set = false;

    // sample and hold event
    if (sample_and_hold_pin == true && sample_and_hold_set == false) {
        sample_position = position;                 // sample position
        position_buffer[0] = sample_position >> 24; // divide sampled position
        position_buffer[1] = sample_position >> 16; // into byte sized chuncks
        position_buffer[2] = sample_position >> 8;  // for SPI transfer
        position_buffer[3] = sample_position;       // 
        sample_and_hold_set = true;                 // set flag that sample has been taken
        PINB |= _BV(0);                             // toggle LED
    } else if (sample_and_hold_pin == false && sample_and_hold_set == true) {
        sample_and_hold_set = false;                // remove sample flag
    }

    // commutate motor
    commutate(phase);
}



void initInterrupt() {

    cli();                //disable interrupts while changing settings
    PCICR |= 1 << PCIE2;  // set bit 2 in PCICR for PCINT23:16 (port D)
    PCMSK2 |= 0xFF;       // enable pin change interrupts on port D (all pins)
    sei();

}



ISR(PCINT2_vect) { // run every time there is a pin change on port D pin change

    // lookup tabel for position incrementing (3-phase version of quadrature)
    static int8_t hall_inc[] = {0,0,0,0,0,0,0,0,0,0,0,1,0,-1,0,0,0,0,0,-1,0,0,1,0,0,-1,1,0,0,0,0,0,0,0,0,0,0,1,-1,0,0,1,0,0,-1,0,0,0,0,0,-1,0,1,0,0,0,0,0,0,0,0,0,0,0};

    // lookup table for setting next motor phase, extended to allow reverse direction
    static uint8_t phase_lookup[] = {0, 2, 4, 3, 6, 1, 5, 2, 4, 3};

    static int hall = 0;
    static uint8_t dir = 0;
    static bool sample_and_hold_set = false;

    int port = PIND; // read the port

    sample_and_hold_pin = port & 1; // flag to tell loop to store current position in buffer
    
    dir = (port >> 1) & 1; // direction value, 1 or 0 
    hall = hall << 3;      // preserve last read for position in/decrement
    hall |= port >> 5;     // shift to read only hall sensors

    if (dir == 0) {
        phase = phase_lookup[(hall & 0x07)];     // determine next phase to fire on the motor
    } else {
        phase = phase_lookup[(hall & 0x07) + 3]; // adding 3 to lookup index has the effect of reversing the motor (MAGIC!)
    }

    position += hall_inc[hall & 0x3F]; // use <hall_prev><hall_current> as lookup index to which will increment, decrement or do nothing to position value

    PINB |= _BV(0);                             // toggle LED
}



ISR(SPI_STC_vect) { // spi handler

    byte position_byte = SPDR; // read in command from master (which byte of position to send next)
    SPDR = position_buffer[position_byte]; // send a single byte from position_buffer

}



void commutate(uint8_t _phase) {

    //                                  {lowers on, Bh:Al,    Ch:Al,    Ch:Bl,    Ah:Bl,    Ah:Cl,    Bh:Cl}
    static uint8_t phase_to_port_ABC[] = {0b000000, 0b010001, 0b100001, 0b100010, 0b001010, 0b001100, 0b010100};
    static uint8_t phase_to_port_ACB[] = {0b000111, 0b100001, 0b010001, 0b010100, 0b001100, 0b001010, 0b100010};
    static uint8_t phase_to_port_BAC[] = {0b000111, 0b001010, 0b100010, 0b100001, 0b010001, 0b010100, 0b001100};
    static uint8_t phase_to_port_BCA[] = {0b000111, 0b001100, 0b010100, 0b010001, 0b100001, 0b100010, 0b001010};
    static uint8_t phase_to_port_CAB[] = {0b000111, 0b100010, 0b001010, 0b001100, 0b010100, 0b010001, 0b100001};
    static uint8_t phase_to_port_CBA[] = {0b000111, 0b010100, 0b001100, 0b001010, 0b100010, 0b100001, 0b010001};

    // the order of the 3 motor wires will dictate which of the above lookup tables to use.
    // TODO: be able to select one of above lookup tables from master
    PORTC = phase_to_port_ABC[_phase];

}
