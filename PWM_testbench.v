`timescale 1ns/1ps

// PWM testbench generates signals to enable simulation of the PWM module.
//  Display the signals clock, reset, data_in and verify that the count is correct.
// Should display three cycles of data_in of 1, 50% and maximum
// change the PWMbits to test more bits.

module PWM_Testbench;

parameter PWMbits = 4;

reg             clk, reset;
reg  [PWMbits-1:0] data_in;
wire PWM;
wire [PWMbits-1:0] count;

PWMunsigned #(.PWM_IN_SIZE(PWMbits)) u1(clk, 1'b1, reset, data_in, PWM, count);
                                            
initial
begin
    clk 	= 1;
    reset 	=0;
end

always // clock
begin
    #10
    clk=~clk;
end

task FundamentalCycleDelay();
	begin
		#(20*(1<<PWMbits));
	end
endtask



always // generate rotations
begin
	#5 reset <= 1'b1;
	#20 reset <= 1'b0; data_in <= 1;
	FundamentalCycleDelay();
	FundamentalCycleDelay();
	FundamentalCycleDelay();
	#20;
	data_in <= (1<< (PWMbits-1));
	FundamentalCycleDelay();
	FundamentalCycleDelay();
	FundamentalCycleDelay();
	#20;
	data_in <= (1<< (PWMbits))-1;
	FundamentalCycleDelay();
	FundamentalCycleDelay();
	FundamentalCycleDelay();
	#100 $stop;
end


endmodule
