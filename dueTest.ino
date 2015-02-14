#include <SPI.h>

int pot = A0;

int ss  = 4;
int dir = 5;
int pwm = 6;
int sh  = 7;



void setup() {

    SerialUSB.begin(9600);

    pinMode(dir, OUTPUT);
    pinMode(pwm, OUTPUT);
    pinMode(pot, INPUT);
    pinMode(13, OUTPUT);

    digitalWrite(dir, HIGH);

    SPI.begin(4);
    SPI.setClockDivider(21); // 84MHz/21 = 4MHz
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    SerialUSB.println("Ready...");

    // pwm channel 0, PWML0 which is pin34 on due
    // activate peripheral instead of gpio
    REG_PIOC_PDR = 4;
    // peripheral B, change PIO_ABSR[2]
    REG_PIOC_ABSR = REG_PIOC_ABSR | 4;

    // pwm settings

    // activate pwm clock (id36), 5th bit of PMC_PCSR1
    REG_PMC_PCER1 = REG_PMC_PCER1 | 16;
    // enable pmw on channel 0
    REG_PWM_ENA = REG_PWM_SR | 1;
    // set period
    REG_PWM_CPRD0 = 4095; // ~20kHz
    // set alignment
    REG_PWM_CMR0 = REG_PWM_CMR0 & 0xFFFFFF8FF;
    // set duty cycle
    REG_PWM_CDTY0 = 0;

}



void loop() {

    const static int vel_avg_points = 100;
    static long position[vel_avg_points];
    static int velocity;
    static unsigned long now;
    static int loop_time = 100;
    static float vel_convert = 1000.0*60.0/float(vel_avg_points*loop_time); // delta position to RPM

    now = millis();

    for (int i=1; i<vel_avg_points; i++) {
        position[i] = position[i-1];
    }

    position[0] = get_position();
    velocity = (position[0] - position[vel_avg_points-1]) * vel_convert;

    int pot_val = analogRead(pot);
    int set = map(pot_val, 0, 1023, -4095, 4095);
    control(set);
    

    SerialUSB.print(set);
    SerialUSB.print(", ");
    SerialUSB.print(position[0]);
    SerialUSB.print(", ");
    SerialUSB.print(velocity);
    SerialUSB.print("\n");

    while (millis() < (now+loop_time)) {}

}



void control(int _pwm_val) {

    if (_pwm_val < 0) {
        digitalWrite(dir, LOW);
    } else {
        digitalWrite(dir, HIGH);
    }

    //analogWrite(pwm, abs(_pwm_val));
    REG_PWM_CDTY0 = abs(_pwm_val);

}



long get_position() {

    static byte p0, p1, p2, p3;
    static unsigned int position_low, position_high;
    static long _position;

    digitalWrite(sh, HIGH);

    digitalWrite(ss, LOW);
    delayMicroseconds(20);
    SPI.transfer(4, 0, SPI_CONTINUE);
    delayMicroseconds(20);
    p0 = SPI.transfer(4, 1, SPI_CONTINUE);
    delayMicroseconds(20);
    p1 = SPI.transfer(4, 2, SPI_CONTINUE);
    delayMicroseconds(20);
    p2 = SPI.transfer(4, 3, SPI_CONTINUE);
    delayMicroseconds(20);
    p3 = SPI.transfer(4, 0);
    digitalWrite(ss, HIGH);

    digitalWrite(sh, LOW);

    position_high = (p0 << 8) | p1;
    position_low  = (p2 << 8) | p3;
    _position  = 0;
    _position |= (long)position_high << 16;
    _position |= position_low;

    return _position;

}
