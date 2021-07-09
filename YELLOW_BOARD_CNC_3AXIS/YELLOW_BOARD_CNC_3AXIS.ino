#include "ModbusSlave.h"
#include "StepperMotor.h"
#include "TAYMAY.h"
#include "Config_slaves.h"
#include <AccelStepper.h>
//================================================================
//================================================================
#define ARDUINO_ADDRESS             3    // Dia chi board arduino slaver can dieu khien

Modbus                  node_slave(MODBUS_SERIAL, ARDUINO_ADDRESS, MODBUS_CONTROL_PIN);       
StepperMotor            Motor_X_ID3, Motor_Y_ID3, Motor_Z_ID3, Motor_A_ID3;

bool coil[32]; uint8_t coil_size = sizeof(coil) / sizeof(coil[0]); // Khai bao số lượng coil dùng cho write single coil
uint16_t state_coil;   // Gia tri tuong ung voi coil[i]
// số xung cài đặt cho ngắt pwm
uint32_t pulse_set_X, pulse_set_Y, pulse_set_Z, pulse_set_A;
// biến trạng thái đã phát xung hoàn tất
bool pulse_done_X , pulse_done_Y, pulse_done_Z, pulse_done_A;
int32_t xung_nguyen[4]; // Lưu số xung trục X,Y,Z,A cần chạy nhận từ modbus
uint32_t pwm_speed[4];  // Lưu tốc độ trục X,Y,Z,A cần chạy nhận từ modbus
// số xung thể hiện vị trí của trục so với điểm zero
int32_t pulse_cnt_A, pulse_cnt_X, pulse_cnt_Y, pulse_cnt_Z;
int32_t pre_pulse_cnt_A, pre_pulse_cnt_X, pre_pulse_cnt_Y, pre_pulse_cnt_Z;
static uint32_t pulse_count_motor_X; 
static uint32_t pulse_count_motor_Y; 
static uint32_t pulse_count_motor_Z; 
static uint32_t pulse_count_motor_A; 
// biến lưu số xung của rotary encoder
static int32_t pulsecounter = 0;
int32_t pre_pulsecounter = 0;
// biến xác định chiều quay của motor
bool state_dir_motor_X, state_dir_motor_Y, state_dir_motor_Z, state_dir_motor_A;
// biến báo đã về vị trí cảm biến gốc máy
bool go_home_X, go_home_Y, go_home_Z, go_home_A;
// giá trị hệ số tần số
int32_t gain_accel_freq_X, gain_accel_freq_Y, gain_accel_freq_Z, gain_accel_freq_A;
int32_t gain_decel_freq_X, gain_decel_freq_Y, gain_decel_freq_Z, gain_decel_freq_A;
uint32_t new_freq_X, new_freq_Y, new_freq_Z, new_freq_A;
static uint16_t accel_count_time_X, accel_count_time_Y, accel_count_time_Z, accel_count_time_A; 
static uint32_t counter_timer1;
static uint8_t state_PB5;
static uint16_t begin_timer1;
//===============================================================================================================
String userInput ="";
static int mycomflag =0;
int32_t pulse_array[] = {1000,3000,-2500,9000,-25600,3200,-25600,-2000,1860,-3000,5000,-7000};
uint8_t pulse_size = sizeof(pulse_array) /sizeof(pulse_array[0]);
uint8_t pwm,dir;
uint8_t rotary_encoder_active;
uint8_t state_pwm = 0;
uint32_t _pulse_run;
uint16_t acceleration_pulse;
#define acc_offset 10
#define acc_dec_pulse 600
#define run_motor_X() bitToggle(PORTB, PB5)
#define set_dir_motor_X(_dir) bitWrite(PORTB, PB6, _dir)
//===============================================================================================================
#define Dir_PWM1A  5
#define pulse_PWM1A      2
#define FREQ_PWM_T01  9000   // tan so phat xung
float _motorMaxSpeed = 20000.0;  //8000.0;
float _motorSpeed = 20000.0;     //8000.0; //pulse per second
float _motorAccel = 100.0;      //87500; //steps/second/second to accelerate
//===============================================================================================================
// Do chọn thanh ghi lưu giá trị so sánh với TCNTn trong thanh ghi ICRn trong phần begin(), cho nên phải chọn ngắt tràn ICRn
//===========================================================================================================
int32_t pulse_packet[] = {25600, 12800, 100, 6400};
uint32_t freq_packet[] = {50000, 10000, 2000, 1500};
uint8_t pack_size = sizeof(pulse_packet)/sizeof(pulse_packet[0]);
bool pulse_done_motor[4];
//===============================================================================================================
ISR (TIMER1_CAPT_vect) // Ngat tran ICR1, ICF1 = 1
{
  pulse_count_motor_X ++;
  if (state_dir_motor_X == CCW) {pulse_cnt_X =  pre_pulse_cnt_X - pulse_count_motor_X;}
  if (state_dir_motor_X == CW)  {pulse_cnt_X =  pre_pulse_cnt_X + pulse_count_motor_X;}

  if (pulse_count_motor_X == pulse_set_X) {stop_motor_X(); }
}
ISR (TIMER3_CAPT_vect) // Ngat tran ICR3, ICF3 = 1
{
  pulse_count_motor_Y ++;
  if (pulse_count_motor_Y < acceleration_pulse) Motor_Y_ID3.execute_acc_dec(1); // tăng tốc
  if (pulse_count_motor_Y > pulse_set_Y - acceleration_pulse) Motor_Y_ID3.execute_acc_dec(-1); // giảm tốc

  if (state_dir_motor_Y == CCW) {pulse_cnt_Y =  pre_pulse_cnt_Y - pulse_count_motor_Y;}
  if (state_dir_motor_Y == CW)  {pulse_cnt_Y =  pre_pulse_cnt_Y + pulse_count_motor_Y;}

  if (pulse_count_motor_Y == pulse_set_Y) {stop_motor_Y();}
}
ISR (TIMER4_CAPT_vect) // Ngat tran ICR4, ICF4 = 1
{
  pulse_count_motor_Z ++;
  if (state_dir_motor_Z == CCW) {pulse_cnt_Z =  pre_pulse_cnt_Z - pulse_count_motor_Z;}
  if (state_dir_motor_Z == CW)  {pulse_cnt_Z =  pre_pulse_cnt_Z + pulse_count_motor_Z;}
  if (pulse_count_motor_Z == pulse_set_Z) {stop_motor_Z(); }
}
ISR (TIMER5_CAPT_vect) // Ngat tran ICR5, ICF5 = 1
{
  pulse_count_motor_A ++;
  if (state_dir_motor_A == CCW) {pulse_cnt_A =  pre_pulse_cnt_A - pulse_count_motor_A;}
  if (state_dir_motor_A == CW)  {pulse_cnt_A =  pre_pulse_cnt_A + pulse_count_motor_A;}

  if (pulse_count_motor_A == pulse_set_A) {stop_motor_A(); }
}
//================================================================
void stop_motor_X(void){
  Motor_X_ID3.detach();
  pre_pulse_cnt_X = pulse_cnt_X;
  pulse_count_motor_X = 0;
  pulse_done_X = true;
  if (go_home_X == true) {pulse_cnt_X = 0; pre_pulse_cnt_X = 0; go_home_X = false;}
}
void stop_motor_Y(void){
  Motor_Y_ID3.detach();
  pre_pulse_cnt_Y = pulse_cnt_Y;
  pulse_count_motor_Y = 0;
  pulse_done_Y = true;
  if (go_home_Y == true) {pulse_cnt_Y = 0; pre_pulse_cnt_Y = 0; go_home_Y = false;}
}
void stop_motor_Z(void){
  Motor_Z_ID3.detach();
  pre_pulse_cnt_Z = pulse_cnt_Z;
  pulse_count_motor_Z = 0;
  pulse_done_Z = true;
  if (go_home_Z == true) {pulse_cnt_Z = 0; pre_pulse_cnt_Z = 0; go_home_Z = false;}
}
void stop_motor_A(void){
  Motor_A_ID3.detach();
  pre_pulse_cnt_A = pulse_cnt_A;
  pulse_count_motor_A = 0;
  pulse_done_A = true;
  if (go_home_A == true) {pulse_cnt_A = 0; pre_pulse_cnt_A = 0; go_home_A = false;}
}
//================================================================
// Thực hiện lệnh DELAY_STEP
void Execute_DelayStep(uint16_t delay_value){
    if (delay_value > 0){delay(delay_value);}
    else delay(1);
}
//================================================================
// Thực hiện lệnh phát xung chạy trục X_AXIS (timer1)
void Execute_XaxisStep(int32_t Xaxis_value, uint32_t freq){
  gain_accel_freq_X = (freq - start_end_freq)/acceleration_time;  // tính hệ số tăng tốc
  gain_decel_freq_X = (start_end_freq - freq)/deceleration_time;  // tính hệ số giảm tốc
  if (Xaxis_value == 0) {pulse_done_X = true;}
  else {
    pulse_done_X = false;
    pulse_set_X = abs(Xaxis_value);
    if (Xaxis_value > 0) {state_dir_motor_X = CW;}
    if (Xaxis_value < 0) {state_dir_motor_X = CCW;}
    Motor_X_ID3.generate_pwm(Xaxis_value, freq);
  }
}
//================================================================
// Thực hiện lệnh phát xung chạy trục Y_AXIS (timer3)
void Execute_YaxisStep(int32_t Yaxis_value, uint32_t freq){
  
  if (Yaxis_value == 0) {pulse_done_Y = true;}
  else {
    pulse_done_Y = false;
    pulse_set_Y = abs(Yaxis_value);
    if (Yaxis_value > 0) state_dir_motor_Y = CW;
    if (Yaxis_value < 0) state_dir_motor_Y = CCW;

    if (pulse_set_Y < 2*acc_dec_pulse + acc_offset ) { 
      acceleration_pulse = (pulse_set_Y - acc_offset)/2;
      if (acceleration_pulse <= 0) acceleration_pulse = 0;
    } 
    else acceleration_pulse = acc_dec_pulse;
    Motor_Y_ID3.generate_pwm(Yaxis_value, freq, acceleration_pulse);
  }
}
//================================================================
// Thực hiện lệnh phát xung chạy trục Z_AXIS
void Execute_ZaxisStep(int32_t Zaxis_value, uint32_t freq){
  gain_accel_freq_Z = (freq - start_end_freq)/acceleration_time;  // tính hệ số tăng tốc
  gain_decel_freq_Z = (start_end_freq - freq)/deceleration_time;  // tính hệ số giảm tốc
  if (Zaxis_value == 0) {pulse_done_Z = true;}
  else {
    pulse_done_Z = false;
    pulse_set_Z = abs(Zaxis_value);
    if (Zaxis_value > 0) {state_dir_motor_Z = CW;}
    if (Zaxis_value < 0) {state_dir_motor_Z = CCW;}
    Motor_Z_ID3.generate_pwm(Zaxis_value, freq);
  }
}
//================================================================
// Thực hiện lệnh phát xung chạy trục A_AXIS
void Execute_AaxisStep(int32_t Aaxis_value, uint32_t freq) { 
  gain_accel_freq_A = (freq - start_end_freq)/acceleration_time;  // tính hệ số tăng tốc
  gain_decel_freq_A = (start_end_freq - freq)/deceleration_time;  // tính hệ số giảm tốc
  if (Aaxis_value == 0) {pulse_done_A = true;}
  else {
    pulse_done_A = false;
    pulse_set_A = abs(Aaxis_value);
    if (Aaxis_value > 0) {state_dir_motor_A = CW;}
    if (Aaxis_value < 0) {state_dir_motor_A = CCW;}
    Motor_A_ID3.generate_pwm(Aaxis_value, freq);
  }
}
//================================================================
// Về gốc máy khi mới mở phần mềm điều khiển
// 2 trục động cơ chạy tới vị trí cảm biến: động cơ xoay ghế, động cơ súng sơn
void go_to_home_position(void) {
  go_home_X = go_home_Y = go_home_Z = go_home_A =  true;
  int32_t pulse_home = 25600;
  Execute_XaxisStep(pulse_home,10000);
  Execute_YaxisStep(pulse_home,10000);
  Execute_ZaxisStep(pulse_home,10000);
  Execute_AaxisStep(pulse_home,10000);
}
//================================================================
// set 0 cho các trục x,y,z,a làm điểm zero position
void set_zero_position(void) {
  pulse_cnt_X = 0; pre_pulse_cnt_X = 0;
  pulse_cnt_Y = 0; pre_pulse_cnt_Y = 0;
  pulse_cnt_Z = 0; pre_pulse_cnt_Z = 0;
  pulse_cnt_A = 0; pre_pulse_cnt_A = 0;
}
//================================================================
// Run main point to point
void execute_point_to_point(int32_t *pulse, uint32_t *speed) {
  Execute_XaxisStep(pulse[0],speed[0]);
  Execute_YaxisStep(pulse[1],speed[1]);
  Execute_ZaxisStep(pulse[2],speed[2]);
  Execute_AaxisStep(pulse[3],speed[3]);
}
//================================================================
// Dừng động cơ ở vị trí bất kỳ
void stop_motor(void) {
  stop_motor_X(); stop_motor_Y(); stop_motor_Z(); stop_motor_A();
}
void pause_motor(void){
  Motor_X_ID3.detach();
  Motor_Y_ID3.detach();
  Motor_Z_ID3.detach();
  Motor_A_ID3.detach();
}
//================================================================
//================================================================
// Thực hiện lệnh INPUT_STEP: ứng với giá trị value -> kiểm tra bit cảm biến tương ứng
// value: giá trị input mong muốn
// wQty: số lượng byte cần đọc từ ngõ input
void Execute_InputStep(uint8_t *value, uint8_t wQty){
    uint8_t *input_data; 
    bool sensor_match; // Biến so sánh giá trị sensor đọc được và value sensor được tạo ra
}
//================================================================
void function_modbus_slave(void){
    node_slave.cbVector[CB_READ_COILS] = readDigital;
    //node_slave.cbVector[CB_READ_DISCRETE_INPUTS] = readDigital;
    node_slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
    //node_slave.cbVector[CB_READ_INPUT_REGISTERS] = readAnalogIn;
    node_slave.cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;
    node_slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemory;
}
//================================================================
// Configure thanh ghi timer/counter 
// http://arduino.vn/bai-viet/411-timercounter-tren-avrarduino
//================================================================
/* 
TCNTn: Thanh ghi thực hiện đếm giá trị 
TCCRnA/B: Thanh ghi điều khiển:
+ CSn0,CSn1,CSn2: Cài đặt prescale
TIMSKn: Thanh ghi điều khiển ngắt. 
+ bit 5 - ICIEn: Input Capture Interrupt Enable - Cho phép ngắt khi dùng Input Capture.
+ bit 2 - OCIEnB: Output Compare Interrupt Enable 1 channel B -  Cho phép ngắt khi dùng Output Compare ở channel B.
+ bit 1 - OCIEnA: Output Compare Interrupt Enable 1 channel A -  Cho phép ngắt khi dùng Output Compare ở channel A.
+ bit 0 - TOIEn: Overflow Interrupt Enable 1 - Cho phép ngắt khi xảy ra tràn trên T/C.
*/
void init_timer_counter_01(){ // Normal mode
// timer 1 tran 65536
  noInterrupts();
  TCCR1B = TCCR1A = 0;
  TIMSK1 = 0;
  TCCR1B |=   (1 << CS11);  // prescaler = 8, P_clock=16mhz/8 = 2mhz, 1 xung = 0.5 us
  //------------------------------------------------------------------------------------------------
  interrupts();  
}
void attach_timer_counter_01(uint16_t init_value){
  noInterrupts();
  TIMSK1 |=  (1 << TOIE1); // Cho phép ngắt khi xảy ra tràn trên T/C.
  TIFR1  |= (0 << TOV1);   // TOV1 = 1 sẽ chạy hàm ngắt
  TCCR1B |=   (1 << CS11);  // prescaler = 8, P_clock=16mhz/8 = 2mhz, 1 xung = 0.5 us
  TCNT1 = init_value; //init_value; 
  begin_timer1 = init_value - acceleration_pulse;
  if (begin_timer1 < 0) begin_timer1 = 0;
  interrupts();
}
void detach_timer_counter_01(){
  noInterrupts();
  TIMSK1 &= ~(1 << TOIE1);
  interrupts();
}
//================================================================
ISR(TIMER1_OVF_vect) 
{ 
  run_motor_X();
  state_pwm = !state_pwm;
  counter_timer1 += state_pwm;
  if (counter_timer1 < acceleration_pulse) begin_timer1 += state_pwm;
  if (counter_timer1 > _pulse_run - acceleration_pulse) begin_timer1 -= state_pwm;
  TCNT1 = begin_timer1; 
  if (counter_timer1 == _pulse_run) {counter_timer1 = 0; detach_timer_counter_01();}
}
//===============================================================================================================
// Ham phat xung: số xung: pulse, tần số: freq, tăng tốc/giảm tốc: acc (theo đơn vị xung)
boolean run_motor_timer1(int32_t pulse, uint32_t freq, uint16_t acc = 500){
  bool pulse_done_PB5 = false; bool dir_motor[4]; 
  uint16_t counter_timer1_motor[4];
/*
  for (int i = 0; i < pack_size, i++){
    if (pulse[i] == 0 || freq[i] == 0){ pulse_done_motor[i] = true;}
    else {
      pulse_done_motor[i] = false;
      dir_motor[i] = (pulse[i] > 0 ? true : false);
      counter_timer1_motor[i] = 1000000/freq[i]; // timer1 đếm 
    }
  }
  */
  if (pulse == 0 || freq == 0) pulse_done_PB5 = true;
  else {
    bool dir_PB6 = (pulse > 0) ? true : false; 
    set_dir_motor_X(dir_PB6);
    uint16_t init_timer1 = 65536 - 1000000/freq; 
    if (init_timer1 < 0) {init_timer1 = 0;}
    _pulse_run = abs(pulse);
    if (_pulse_run < 2*acc + acc_offset ) { 
      acceleration_pulse = (_pulse_run - acc_offset)/2;
      if (acceleration_pulse <= 0) acceleration_pulse = 0;
    } 
    else acceleration_pulse = acc;
    attach_timer_counter_01(init_timer1); // bắt đầu đếm thời gian dùng timer 1
  }
  Serial.println("pulse_done_PB5");
  return pulse_done_PB5;
}

//===============================================================================================================
// Read the rotary encoder
void encoder() {
  bool state_encoder_channel_B = PINA&B00000010; //digitalRead(EncoderB);
  switch (state_encoder_channel_B) {
    case 0: pulsecounter--; bitClear(PORTB, PB6); break;
    case 1: pulsecounter++; bitSet(PORTB, PB6); break; 
    default: break;       
  }
  
}
// Enable interrupts number 4
void attach_rotary_encoder(void) {
  attachInterrupt(4, encoder, RISING); 
  pulsecounter = 0;
  pre_pulsecounter = 0;
  rotary_encoder_active = 1;
}
// Disable interrupts number 4
void detach_rotary_encoder(void) {
  detachInterrupt(4); // tắt ngắt 4
  pulsecounter = 0;
  pre_pulsecounter = 0;
  rotary_encoder_active = 0;
}
//===============================================================================================================
//============================================================================================
void init_motor_id3(void){
  noInterrupts();
  Motor_Z_ID3.begin(PWM_MOTOR_03, DIR_MOTOR_03, ILDE_MOTOR_03);
  Motor_Y_ID3.begin(PWM_MOTOR_02, DIR_MOTOR_02, ILDE_MOTOR_02);
  Motor_X_ID3.begin(PWM_MOTOR_01, DIR_MOTOR_01, ILDE_MOTOR_01);
  Motor_A_ID3.begin(PWM_MOTOR_04, DIR_MOTOR_04, ILDE_MOTOR_04);
  interrupts();
}
void reset_slave3(void){
  asm volatile ( "jmp 0"); 
}
//================================================================================================
void run_array_pulse(void){
  for (int i = 0; i < 12; i++){
    Execute_XaxisStep(pulse_array[i],1000);
    while (true){ 
      delay(1);
      if (pulse_done_X == true){ break;}
    }
  }
}
//================================================================================================
void go_to_home(void){
  go_home_X = true;
  int32_t pulse_home = -1*pulse_cnt_X;
  Serial.print("gia tri pulse home: "); Serial.println(pulse_home); 
  Execute_XaxisStep(pulse_home, 100000);
}
//================================================================================================
void init_pin_motor (uint8_t pwm_pin, uint8_t dir_pin){
    pwm = pwm_pin; dir = dir_pin;
    pinMode(pwm, OUTPUT); pinMode(dir, OUTPUT);
    digitalWrite(dir, LOW); digitalWrite(pwm, LOW);
}
//================================================================================================
void Run_motor(int32_t Xaxis_value, uint32_t freq){
  pulse_cnt_X = 0; 
  if (Xaxis_value < 0){ digitalWrite(dir, LOW);}
  if (Xaxis_value > 0){ digitalWrite(dir, HIGH);}
  while (pulse_cnt_X != abs(Xaxis_value)){
    digitalWrite(pwm,HIGH); delayMicroseconds(30);
    digitalWrite(pwm,LOW);  delayMicroseconds(30);
    pulse_cnt_X ++;
  }
}
//================================================================================================
/*
void run_stepper_lib(void){
  for ( int sr = 0; sr < 2; sr++){
    delay(100);
    stepper.move(800);
    stepper.runToPosition();
  }  
    delay(100);
    stepper.move(3200);
    stepper.runToPosition();
    delay(1000);
}
*/
//================================================================================================
//================================================================================================
//============================================================================================
void setup() {   
  Serial.begin(9600);
  MODBUS_SERIAL.begin(MODBUS_BAUDRATE);
  node_slave.begin(MODBUS_BAUDRATE);
  pulse_done_X = pulse_done_Y = pulse_done_Z = pulse_done_A = true;
  Motor_Y_ID3.begin(PWM_MOTOR_02, DIR_MOTOR_02, ILDE_MOTOR_02);
  //init_pin_motor(pulse_PWM1A,Dir_PWM1A);
  //init_motor_id3();
  //function_modbus_slave();
  //stepper.setMaxSpeed(_motorMaxSpeed);
  //stepper.setSpeed(_motorSpeed);
  //stepper.setAcceleration(_motorAccel);
  //DDRB |= (1 << PB5); PORTB |=(1<<PB5);
  //pinMode (11, OUTPUT); digitalWrite(11,LOW);
  //pinMode (12, OUTPUT); digitalWrite(12,LOW);
  pinMode(EncoderA, INPUT_PULLUP);
  pinMode(EncoderB, INPUT_PULLUP);
  //init_timer_counter_01();
  Serial.println("Setup OK");
}
//============================================================================================
//============================================================================================
void loop() { 
  // nên đưa hàm poll vào vòng loop trong trường hợp monitor data về máy tính. không nên dùng ngắt để chạy poll. 
  //node_slave.poll();
  //Serial.println(pulsecounter);
  //Serial.println(counter_timer1);
  
  if (rotary_encoder_active == 1){
      bool state_encoder_channel_A = PIND&B00000100;
      bitWrite(PORTB, PB5, state_encoder_channel_A);
  }
  //if (mycomflag == 1) {bool result = run_motor_timer1(12800, 50000); mycomflag = 0;}
  //if (mycomflag == 2) {bool result = run_motor_timer1(-12800, 50000); mycomflag = 0;}
  if (mycomflag == 1) {Execute_YaxisStep(25600, 20000); mycomflag = 0;}
  if (mycomflag == 2) {Execute_YaxisStep(-25600, 20000); mycomflag = 0;}
  userInput = "";
  while (Serial.available()>0){
    userInput += char(Serial.read()); 
  
    delay(2);
 
    if (userInput ==  "f")
      {
        Serial.println ("FORWARD STEPPER MOTOR");
        mycomflag = 1;
        userInput="";
      }  
    if (userInput ==  "r")
      {
        Serial.println ("REVERSE STEPPER MOTOR");
        mycomflag = 2;
        userInput="";
      }

    if (userInput ==  "home"){
      Serial.println ("GOTO HOME");
      userInput="";
    }
  }

} // End loop
//============================================================================================
//============================================================================================
uint32_t *convert_32bit_data(uint32_t *data, uint16_t length){
  static uint32_t output_data[32];
  for (int i = 0; i <  length/2; i++){
      output_data[i] = data[2*i] << 16 | data[2*i+1];
  }
  return output_data;
}
//============================================================================================
// Handle the function codes Write Holding Register(s) (FC=06, FC=16) and write data to the eeprom.
// Ghi giá trị vào eeprom địa chỉ bắt đầu từ address. số lượng data 16 bit: length
//============================================================================================
uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length)
{
  uint32_t read_data[32];
  int32_t *i32readdata; 
  if (address < 0 || (address + length) > 32) { return STATUS_ILLEGAL_DATA_ADDRESS; }

  for (int i = 0; i < length; i++) {
    read_data[i] = node_slave.readRegisterFromBuffer(i);
  }
  i32readdata = convert_32bit_data(read_data,length);
  switch (address) {
    // Địa chỉ 0 nhận số xung cần chạy cho trục x,y,z,a
    case PULSE_EXECUTE: for (int i = 0; i < 4; i++){ xung_nguyen[i] = i32readdata[i];}  break;   
    // Địa chỉ 10 nhận tần số phát xung cho trục x,y,z,a
    case FREQ_EXECUTE: for (int i = 0; i < 4; i++){ pwm_speed[i] = i32readdata[i];}  break;
    default: break;
  }
  return STATUS_OK;     
}
//============================================================================================
// Handle the function code Read Holding Registers (FC=03) and write back the values from the EEPROM (holding registers).
//============================================================================================
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length)
{
    uint16_t value[length]; 
    if (address < 0 ||(address + length) > 32) { return STATUS_ILLEGAL_DATA_ADDRESS; }
    switch (address) {
      case PWM_VALUE_SPRAY_AXIS_MODBUS_ADDR: 
            value[0] = highWord(pulse_cnt_X);
            value[1] = lowWord(pulse_cnt_X); 
            break;
      case PWM_VALUE_CHAIR_AXIS_MODBUS_ADDR:
            value[0] = highWord(pulse_cnt_Y);
            value[1] = lowWord(pulse_cnt_Y); 
            break;
      case PWM_VALUE_Z_AXIS_MODBUS_ADDR:  // Không dùng
            value[0] = highWord(pulse_cnt_Z);
            value[1] = lowWord(pulse_cnt_Z); 
            break;
      case PWM_VALUE_A_AXIS_MODBUS_ADDR:  // Không dùng
            value[0] = highWord(pulse_cnt_A);
            value[1] = lowWord(pulse_cnt_A);
            break;
    }
    for (int i = 0; i < length; i++){  
        node_slave.writeRegisterToBuffer(i, value[i]); // Write 16 bit data
    }
  return STATUS_OK;
}
//============================================================================================
// Handle the function codes Force Single coil (FC=05) and Force Multiple Coils (FC=15) and set the digital output pins (coils).
//============================================================================================
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length)
{
  uint16_t address_index = address;
  // Check if the requested addresses exist in the array
  if (address > coil_size || (address + length) > coil_size) { return STATUS_ILLEGAL_DATA_ADDRESS; }
  // Set the output pins to the given state
  for (int i = 0; i < length; i++) {
      if (address + i < COIL_Y1_FIRST_MODBUS_ADDR) { // địa chỉ nhỏ hơn 32 thì nhận lệnh force single coil
        coil[address+i] = node_slave.readCoilFromBuffer(i);
        if (coil[address+i] == 1) {
            switch (address + i) {
              //case SPRAY_ONOFF_MODBUS_ADDR: test_4a_id3(); break;
              case POINT2POINT_MODBUS_ADDR:   execute_point_to_point(xung_nguyen,pwm_speed); break;
              case PAUSE_MOTOR_MODBUS_ADDR:   pause_motor(); break;
              case ENABLE_HOME_MOBUS_ADDR:    go_to_home_position(); break;  // về vị trí cảm biến gốc máy
              case SET_ZERO_POSITION_ADDR:    set_zero_position(); break;    // set 0 tọa độ chương trình
              case STOP_MOTOR_MODBUS_ADDR:    stop_motor(); break;
              default: break;
            }
        } 
      }
  }
  return STATUS_OK;
}
//============================================================================================
// Handle the function codes Read Input Status (FC=01/02) and write back the values from the digital pins (input status).
uint8_t readDigital(uint8_t fc, uint16_t address, uint16_t length)
{
  // Check if the requested addresses exist in the array
  if (address > coil_size || (address + length) > coil_size) { return STATUS_ILLEGAL_DATA_ADDRESS; }

  if (address == EXECUTE_PULSE_DONE) {
    bool state_home = (pulse_done_X & pulse_done_Y & pulse_done_Z & pulse_done_A);
    for (int i = 0; i < length; i++) {
        // Write the state of the digital pin to the response buffer.
        node_slave.writeCoilToBuffer(i,state_home);
    }
  }
  return STATUS_OK;
}
//============================================================================================
//end
