// This code is provided as an example of how to communicate with the
// Smart BLDC Commutator board. It was written to run on an
// Arduino Due. Some changes will be required to run on other
// Arduino varients.

byte clr;
int sample_and_hold_pin = 8;
int pwm_pin = 9;
int pwm_val = 0;
int pot_pin = A5;
int dir_pin = 7;


void setup() {

    Serial.begin(9600);

    pinMode(sample_and_hold_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
    pinMode(pot_pin, INPUT);
    pinMode(dir_pin, OUTPUT);

    digitalWrite(dir_pin, LOW);
    digitalWrite(sample_and_hold_pin, LOW);
    analogWrite(pwm_pin, 0);

    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);
    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);

	// Due settings to change PWM frequency and set up SPI (not using SPI libraries here)
	// Arduino Uno won't be able to change PWM frequency much, but the SPI lines should
	// be pretty close to these. Read the datasheet!
    SPCR |= 0b01;       // SPI speed f_osc/16 (1MHz with 16MHz chip)
    SPCR &= ~_BV(DORD); // MSB first
    SPCR |=  _BV(MSTR); // master mode
    SPCR &= ~_BV(CPOL); // clock low when idle
    SPCR &= ~_BV(CPHA); // sample on clock rising edge
    SPCR |=  _BV(SPE);  // enable SPI
    SPCR &= ~_BV(SPIE); // disable SPI interrupts (master mode)

    clr = SPSR;
    clr = SPDR;
    delay(10);

    Serial.println("Ready...");

}



void loop() {

    static long position, last_position;
    static int velocity;
    static unsigned long now;
    static int loop_time = 100;
    static float vel_convert = 1000.0*60/(66*loop_time); // delta position to RPM
    char* foo;

    now = millis();

    last_position = position;
    position = get_position(); // communication with BLDC in here
    velocity = (position - last_position) * vel_convert; 

    int set = map(analogRead(pot_pin),0,1023,-254, 255);
    control(set);

    Serial.print(set);
    Serial.print(", ");
    Serial.print(position);
    Serial.print(", ");
    Serial.print(velocity);
    Serial.print("\n");


    while (millis() < (now+loop_time)) {}

}



void control(int _set) {

    if (_set >= 0) {
        digitalWrite(dir_pin, HIGH);
    } else {
        digitalWrite(dir_pin, LOW);
    }

    analogWrite(pwm_pin, abs(_set));

}



byte spi_transfer(byte data) {

    SPDR = data; // put byte in the SPDR register to be sent out to slave
    while (!(SPSR & (1<<SPIF))) {}

    return SPDR; // return what was read in from slave
}



long get_position() {

    static byte p0, p1, p2, p3;
    static unsigned int position_low, position_high;
    static long _position;
    
    digitalWrite(sample_and_hold_pin, HIGH); // tell the slave to hold current value for sampling

    digitalWrite(SS, LOW);  // pull slave select low to init communication
    delayMicroseconds(20);  // wait after every SPI command
    spi_transfer(0);        // send nothing to get SPI pipline primed
    delayMicroseconds(20);
    p0 = spi_transfer(1);   // first byte of position
    delayMicroseconds(20);
    p1 = spi_transfer(2);   // second byte
    delayMicroseconds(20);
    p2 = spi_transfer(3);   // third byte
    delayMicroseconds(20);
    p3 = spi_transfer(0);   // fourth byte
    digitalWrite(SS, HIGH); // end communication

    digitalWrite(sample_and_hold_pin, LOW); // release hold on slave

	// union the four bytes into one long value
    position_high = (p0 << 8) | p1;
    position_low  = (p2 << 8) | p3;
    _position = 0;
    _position |= (long)position_high << 16;
    _position |= position_low;

    return _position;

}


