// MODULE InterruptLatencyEventTiming
//   *  Use this module to measure interrupt latency and other signal timing to the nearest microsecond with rounding.
//   *  Interrupts are assumed to occur when either EncPhA_in or EncPhB_in changes.
//   *  The time from the first interrupt event since rst_interrupt_timer_in 1->0 is available on microseconds_out.
//   *  The duration ext_in=1 starting from the rst_ext_timer_in 1->0 is available on microseconds_out.
//   *  The mode of measuring interrupt latency or external signal duration is determined by which is the last reset action:
//      If rst_ext_timer_in=1->0 occurs last then ext signal timing mode is used, 
//      if it is rst_interrupt_timer_in=1->0 then interrupt latency mode is used.
//   *  To measure interrupt latency, connect the A/B encoder signals to EncPhA_in/EncPhB_in.
//          In software, issue a rst_interrupt_timer_in = 1 and then 0 to initialise via a parallel port.  Let's call this port "ControlPort".
//          (We are assuming ControlPort has 3 outputs:  rst_interrupt_timer_in, rst_ext_timer_in and ext_in).
//          It is assumed that interrupts are configured to occur on a change in the A or B encoder signals.
//          Lets assume the microseconds_out of this module connects to a 32 bit parallel port called "MicroPort".
//          Within the encoder Interrupt Service Routine (ISR) for the A/B encoder signals,
//          read the MicroPort parallel port to obtain the interrupt latency and then issue a rst_interrupt_timer_in = 1 via the ControlPort.
//          Don't forget to restore rst_interrupt_timer_in to 0 via ControlPort afterwards (still in the ISR)!
//   *  Using ext_in allows timing of durations of events.  This is how:
//          Connect ext_in to the signal that you wish to measure its duration high.  This could be an internal logic signal or the ext_in output 
//             of the ControlPort (defined above) that software controls in the case where you wish to measure software execution times.
//          Using the ControlPort, issue a rst_ext_timer_in=1 followed by rst_ext_timer_in=0 to initialise external signal mode (as opposed to interrupt mode).
//          When ext_in is 1 the timer then increments and when ext_in is 0 it pauses.
//          Read the MicroPort parallel port for the current accumulated time.
//          Reset the time by applying rst_ext_timer_in=1 followed by 0 via the ControlPort.
//   *  The clk_in is assumed to be 50 Mhz.  Time measurements are rounded to the nearest microsecond.
//   *  Example of instantiation of this module:
//          TimeFromLastInterrupt local_name(enA, enB, extsignal, CLOCK_50, rst_interrupt_timing, rst_ext_timing, microseconds);
//   ********************************************************************************************************    

module InterruptLatencyEventTiming(EncPhA_in, EncPhB_in, ext_in, clk_in /* 50 Mhz*/, rst_interrupt_timer_in, rst_ext_timer_in, microseconds_out);
    input EncPhA_in, EncPhB_in, clk_in,rst_interrupt_timer_in, rst_ext_timer_in, ext_in;
    output [31:0] microseconds_out;
	 reg [31:0] microseconds;
    reg gate;          // when 1 timer increments
    reg prevA, prevB;  // previous encoder signals
    reg mode;   // when mode = 1 time since last encoder interrupt is timed, mode = 0 ext_in = 1 is timed

	 assign microseconds_out = microseconds;
	 
    always@(posedge clk_in) begin
        if (rst_interrupt_timer_in || rst_ext_timer_in) 
            gate  <= 1'b0;
        if (rst_ext_timer_in) 
            mode <= 1'b0;
        if (rst_interrupt_timer_in) 
            mode <= 1'b1;
            
        if (mode) begin  // interrupt timing mode
            if (prevA != EncPhA_in  || prevB != EncPhB_in)
                gate <= 1'b1;  // an interrupt is assumed generated!
        end else begin // mode where ext_in enables counting - this allows accumulation of multiple durations until rst_ext_timer_in=1
            gate <= ext_in;
        end
        prevA <= EncPhA_in;
        prevB <= EncPhB_in;
    end

    reg [5:0] CEcount;
    wire CE_micro = (CEcount==49);
    // wire CE_micro = (CEcount==4);  // enable this line instead of the one above if you want to measure time in units of 1/10 usec
    always@(posedge clk_in)
        if (rst_interrupt_timer_in || rst_ext_timer_in)
                CEcount <= 24;  // so that rounding to nearest usec occurs
        else if (CE_micro) 
                CEcount <= 0;
            else if (gate)
                CEcount <= CEcount + 1'b1;
        
    always@(posedge clk_in) begin
        if (rst_interrupt_timer_in || rst_ext_timer_in) 
            microseconds <= 0;
        else if (CE_micro)
            microseconds <= microseconds+1'b1;
    end

endmodule