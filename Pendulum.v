module Pendulum
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_50,						//	50 MHz
		////////////////////	Push Button		////////////////////
		KEY,							//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digit 0
		HEX1,							//	Seven Segment Digit 1
		HEX2,							//	Seven Segment Digit 2
		HEX3,							//	Seven Segment Digit 3
		HEX4,							//	Seven Segment Digit 4
		HEX5,							//	Seven Segment Digit 5
		HEX6,							//	Seven Segment Digit 6
		HEX7,							//	Seven Segment Digit 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		// MICROSECONDS, // Importing microsecond module within this module, don't need externally any more
		//TIMERCONTROL, Keeping Interrupt module within this module so don't need externally any more
		Hbridge_InA,
		Hbridge_PWM,
		Hbridge_InB,
		enAin,
		enBin,
		beam
	);

////////////////////////	Clock Input	 	////////////////////////
input			CLOCK_50;				//	50 MHz
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;					//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	SW;						//	Toggle Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digit 0
output	[6:0]	HEX1;					//	Seven Segment Digit 1
output	[6:0]	HEX2;					//	Seven Segment Digit 2
output	[6:0]	HEX3;					//	Seven Segment Digit 3
output	[6:0]	HEX4;					//	Seven Segment Digit 4
output	[6:0]	HEX5;					//	Seven Segment Digit 5
output	[6:0]	HEX6;					//	Seven Segment Digit 6
output	[6:0]	HEX7;					//	Seven Segment Digit 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;					//	LED Green[8:0]
output	[17:0]	LEDR;					//	LED Red[17:0]
//input		[31:0] MICROSECONDS; // Importing microsecond module within this module, don't need externally any more
//output 	[2:0] TIMERCONTROL;

(* chip_pin = "AB22" *) output Hbridge_InA; //GPIO[0]
(* chip_pin = "AC15" *) output Hbridge_PWM; //GPIO[1]
(* chip_pin = "AB21" *) output Hbridge_InB; //GPIO[2]
//output Hbridge_InA; //GPIO[0]
//output Hbridge_PWM; //GPIO[1]
//output Hbridge_InB; //GPIO[2]
(* chip_pin = "Y17" *) input enAin; //GPIO[3]
(* chip_pin = "AC21" *) input enBin; //GPIO[4]
(* chip_pin = "Y16" *) input beam; //GPIO[5]


assign LEDR[9:0] = pid_signal;
parameter PWM_IN_SIZE = 10; // this is a constant that can be overridden when 

	 
LK u0 (
        .clk_clk             (CLOCK_50),             //            clk.clk
	.angle_export        (encoder_angle),
        .encoder_export      ({enAin, enBin}),      //        encoder.export
        .hexdisplay3to0_HEX0 (HEX0), // hexdisplay3to0.HEX0
        .hexdisplay3to0_HEX1 (HEX1), //               .HEX1
        .hexdisplay3to0_HEX2 (HEX2), //               .HEX2
        .hexdisplay3to0_HEX3 (HEX3), //               .HEX3
        .hexdisplay7to4_HEX4 (HEX4), // hexdisplay7to4.HEX4
        .hexdisplay7to4_HEX5 (HEX5), //               .HEX5
        .hexdisplay7to4_HEX6 (HEX6), //               .HEX6
        .hexdisplay7to4_HEX7 (HEX7), //               .HEX7
        .keys_export         (KEY),         //           keys.export
        .ledgreen_export     (LEDG),     //       ledgreen.export
        //.ledred_export       (LEDR),       //         ledred.export
        .microseconds_export (microseconds), //   microseconds.export
        .slidersw_export     (SW),     //       slidersw.export
        .timercontrol_export (timercontrol),  //   timercontrol.export
		  //.hbridge_export      ({Hbridge_InA,Hbridge_InB}),      //        hbridge.export
        .hbridge_pwm_export  (pid_signal),   //    hbridge_pwm.export data_input_pwm_export
		  //.hbridge_pwm_export  (pwm),   //    hbridge_pwm.export
		  .beam_export         (beam)          //           beam.export
    );

//assign LEDR = SW;  // if you want this, disable .ledred_export line above

// Import hardware encoder angle module
wire [31:0] encoder_angle;
EncoderAngle enc (.CLK_50(CLOCK_50), .enAin(enAin), .enBin(enBin), .reset(KEY[0]), .count_out(encoder_angle));

// Import microsecond generating module and measure ISR latency
wire [31:0] microseconds;
wire 	[2:0] timercontrol;

// Generate microseconds
MicrosecondGenerator usec (.CLK_50(CLOCK_50), .microseconds(microseconds), .reset(KEY[0]));

// InterruptLatencyEventTiming lat (.EncPhA_in(enAin), .EncPhB_in(enBin), .ext_in(timercontrol[2]), .clk_in(CLOCK_50) /* 50 Mhz*/,
 // .rst_interrupt_timer_in(timercontrol[1]), .rst_ext_timer_in(timercontrol[0]), .microseconds_out(microseconds));

// Change clock speed to be appropriate for 10kHz PWM fundamental frequency 
wire clk_pwm;
wire [PWM_IN_SIZE - 1:0] pid_signal;
wire pwm;
ClockChanger clk_changer (.clk_in(CLOCK_50), .CE_out(clk_pwm));



// Unsigned Version
//PWMunsigned pwm_mod (.clk_in(CLOCK_50), .CE_in(clk_pwm), .synch_reset_in(~KEY[0] & CLOCK_50), .PWM_data_input(pwm), .PWM_out(pwm));
//PWMsigned pwm_mod (.clk_in(CLOCK_50), .CE_in(clk_pwm), .synch_reset_in(KEY[0] & CLOCK_50), .PWM_data_input(SW[9:0]), .PWM_out(pwm));
//assign Hbridge_PWM = pwm;
//assign Hbridge_InA = 1'b1;
//assign Hbridge_InB = 1'b0;

//PWMsigned pwm_mod (.clk_in(CLOCK_50), .CE_in(clk_pwm), .synch_reset_in(KEY[0] & CLOCK_50), .PWM_data_input(SW[PWM_IN_SIZE - 1:0]), .PWM_out(pwm));
PWMsigned pwm_mod (.clk_in(CLOCK_50), .CE_in(clk_pwm), .synch_reset_in(KEY[0] & CLOCK_50), .PWM_data_input(pid_signal), .PWM_out(pwm));
assign Hbridge_PWM = pwm;
assign Hbridge_InA = pid_signal[PWM_IN_SIZE - 1];
assign Hbridge_InB = ~pid_signal[PWM_IN_SIZE - 1];
 
endmodule