module MicrosecondGenerator(CLK_50, microseconds, reset);

input CLK_50;
input reset;
reg [31:0] CEcount;
wire CE_micro = (CEcount == 32'd49);
output reg [31:0] microseconds;

always @(posedge CLK_50)
	if (CE_micro) CEcount <= 0;
	else CEcount <= CEcount + 1'b1;


always @(posedge CLK_50)
	if (~reset) microseconds <= 0;
	else if (CE_micro) microseconds <= microseconds + 1'b1;
	
endmodule