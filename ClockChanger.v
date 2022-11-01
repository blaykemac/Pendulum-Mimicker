module ClockChanger(clk_in, CE_out);
parameter CLOCK_PERIODS = 32'd10; // This will take original clock speed = C Hz, then produce output clock = C / CLOCK_PERIODS
 // Instantiating this module
input clk_in;
reg [31:0] count = 0;
output wire CE_out = (count == CLOCK_PERIODS - 1);

always @(posedge clk_in) begin
//if (reset) // may need reset later
	if (CE_out) count <= 0;
	else count <= count + 1'b1;
	
end

endmodule