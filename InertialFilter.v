module InertialFilter(clk, CE, synch_reset, data_in, data_out);
input clk, CE, data_in, synch_reset;
output reg data_out;

localparam CONSEC_COUNT = 10;

reg [3:0] count;

always @(posedge clk)
if (CE) begin
	if (synch_reset) begin
			count <= 0;
			data_out <= data_in;
	end else if (data_out == data_in) 
			count <= 0;
		else if (count == CONSEC_COUNT-1) begin
			data_out <= data_in;
			count <= 0;
		end else
			count <= count + 1'b1;
end

endmodule