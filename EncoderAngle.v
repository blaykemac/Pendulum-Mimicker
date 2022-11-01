module EncoderAngle (input CLK_50, input enAin, input enBin, input reset, output wire [31:0] count_out);
reg As, Bs; // synchronised A, B
reg A_prev, B_prev;
reg [31:0] count;
// Need to make count 32 bits?
assign count_out = count;
//assign count_out = 32'bffffffff;


always@(posedge CLK_50) begin
	// synchroniser with one clock period
	// metastable settling time
 	As <= enAin; Bs <= enBin;

	// previous update
	A_prev <= As; B_prev <= Bs;
end

always @(posedge CLK_50)
	if (~reset) count <= 0;
 	else case({A_prev, B_prev, As, Bs})
		4'b0010, 4'b1011, 4'b1101, 4'b0100: 
			count <= count + 1'b1; 
		4'b0001, 4'b0111, 4'b1110, 4'b1000: 
			count <= count - 1'b1;
	default: ;// do nothing since double 
	// transition - could flag error here
	

 endcase

endmodule