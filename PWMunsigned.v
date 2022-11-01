module PWMunsigned(clk_in, CE_in, synch_reset_in, PWM_data_input, PWM_out); //, count);
parameter PWM_IN_SIZE = 10; // this is a constant that can be overridden when 
 // Instantiating this module
input clk_in, CE_in, synch_reset_in; 
// synch_reset_in must be synchronised to clk_in
input [PWM_IN_SIZE-1:0] PWM_data_input; 
output reg PWM_out; 
// out=1 in proportion to magnitude of 
// PWM_data_input/2**[PWM_IN_SIZE] 
//output reg unsigned[PWM_IN_SIZE-1:0] count; 
reg unsigned[PWM_IN_SIZE-1:0] count;
// … other local signals declared here
reg [PWM_IN_SIZE-1:0] PWM_data_input_buffered;
always @(posedge clk_in)
if (synch_reset_in) begin
	count <= 0;
	PWM_data_input_buffered <= PWM_data_input;
	PWM_out <= 0;
	end
else if (CE_in) begin
	count <= count + 1'b1;
	//PWM_out <= 0;
	PWM_out <= (count < PWM_data_input_buffered);
	if (count == (2**PWM_IN_SIZE - 1)) begin
	PWM_data_input_buffered <= PWM_data_input;
	count <= 0;
	
end
end
endmodule