module PWMsigned(clk_in, CE_in, synch_reset_in, PWM_data_input, PWM_out); //, count);
parameter PWM_IN_SIZE = 10; // this is a constant that can be overridden when 
 // Instantiating this module
input clk_in, CE_in, synch_reset_in; 
// synch_reset_in must be synchronised to clk_in
input signed [PWM_IN_SIZE-1:0] PWM_data_input; 
output reg PWM_out; 

// out=1 in proportion to magnitude of 
// PWM_data_input/2**[PWM_IN_SIZE] 
//output reg unsigned[PWM_IN_SIZE-1:0] count; 
reg [PWM_IN_SIZE-2:0] count;
// … other local signals declared here
reg signed [PWM_IN_SIZE-1:0] PWM_data_input_buffered;

wire [PWM_IN_SIZE-1:0] magn = (PWM_data_input[PWM_IN_SIZE-1]? 1'b0 - PWM_data_input: PWM_data_input);


always @(posedge clk_in)
if (synch_reset_in) begin
	count <= 0;
	PWM_data_input_buffered <= magn;
	PWM_out <= 0;
	end
else if (CE_in) begin
	count <= count + 1'b1;
	//PWM_out <= 0;
	PWM_out <= (count < PWM_data_input_buffered);
	if (count == (2**(PWM_IN_SIZE - 1) - 1)) begin
	PWM_data_input_buffered <= magn;
	count <= 0;
	
end
end
endmodule