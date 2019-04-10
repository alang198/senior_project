module sync(

	input clk_fpga,
	input clk_ps2,
	input flag_in,
	output reg flag_out
);

reg flag_a;

always@(posedge clk_ps2) flag_a <= flag_a ^ flag_in; //toggles on flag assertion

reg [2:0] sync_to_main;
always@(posedge clk_fpga) sync_to_main <= {sync_to_main[1:0], flag_a}; //move to 50MHz clock

assign flag_out = sync_to_main[2] ^ sync_to_main[1];

endmodule 