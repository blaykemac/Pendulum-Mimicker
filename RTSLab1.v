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
		LEDR							//	LED Red[17:0]
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


//	Turn on all display
//assign	HEX0		=	7'h00;
//assign	HEX1		=	7'h00;
//assign	HEX2		=	7'h00;
//assign	HEX3		=	7'h00;
//assign	HEX4		=	7'h00;
//assign	HEX5		=	7'h00;
//assign	HEX6		=	7'h00;
//assign	HEX7		=	7'h00;
//assign	LEDG		=	9'h1FF;
//assign	LEDR		=	18'h3FFFF;

LK u0 (
        .clk_clk                        (CLOCK_50),  //                       clk.clk
        .slidersw_export                (SW),        //                  slidersw.export
		  .hexdisplay3to0_HEX0            (HEX0),      //            hexdisplay3to0.HEX0
        .hexdisplay3to0_HEX1            (HEX1),      //                          .HEX1
        .hexdisplay3to0_HEX2            (HEX2),      //                          .HEX2
        .hexdisplay3to0_HEX3            (HEX3),      //                          .HEX3
        .hexdisplay7to4_HEX4            (HEX4),      //            hexdisplay7to4.HEX4
        .hexdisplay7to4_HEX5            (HEX5),      //                          .HEX5
        .hexdisplay7to4_HEX6            (HEX6),      //                          .HEX6
        .hexdisplay7to4_HEX7            (HEX7),      //                          .HEX7
        .ledgreen_export                (LEDG),      //                  ledgreen.export
        .ledred_export                  (LEDR),      //                    ledred.export
        .keys_export                    (KEY)        //                      keys.export
    );

//assign LEDR = SW;  // if you want this, disable .ledred_export line above

endmodule