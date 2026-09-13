`timescale 1ns / 1ps
//======================================================================
//  stage2_design.v   --   Stage 2 of async_fifo_cdc
//
//  All three Stage 2 design modules in one file:
//
//      bin2gray   binary  -> Gray            (combinational)
//      gray2bin   Gray    -> binary          (combinational)
//      sync_2ff   two-flip-flop synchronizer (sequential)
//
//  Testbench for these lives in stage2_tb.v
//
//  NOTE: if you previously added bin2gray.v, gray2bin.v and sync_2ff.v
//  to your project, REMOVE them before adding this file.  Two copies of
//  the same module name will make the tools complain.
//======================================================================

//======================================================================
//  MODULE 1 of 3 :  bin2gray
//
//  Converts a binary value to its Gray-code equivalent.
//
//  The whole thing is one line.  The rule is:
//
//      gray = bin XOR (bin >> 1)
//
//  i.e. each Gray bit is the XOR of the binary bit at that position and
//  the binary bit one position above it.  The top bit passes straight
//  through, because there is nothing above it (the shift brings in a 0).
//
//  Worked example, 4 bits, bin = 4'b0110 (6):
//
//        bin        0 1 1 0
//        bin >> 1   0 0 1 1
//        XOR        -------
//        gray       0 1 0 1
//
//  The property that matters for CDC:  when a counter increments, its
//  Gray code changes in EXACTLY ONE BIT.  Compare:
//
//        count   binary    gray     bits changed in gray
//          6      0110     0101
//          7      0111     0100          1   (bit 0)
//          8      1000     1100          1   (bit 3)
//
//  Going from 7 to 8 in binary flips all four bits at once.  In Gray it
//  flips one.  Stage 2's testbench shows why that difference decides
//  whether a value can safely cross a clock domain.
//
//  This is purely combinational -- no clock, no reset, no state.
//======================================================================

module bin2gray #(
    parameter WIDTH = 5
)(
    input  wire [WIDTH-1:0] bin,
    output wire [WIDTH-1:0] gray
);

    assign gray = bin ^ (bin >> 1);

endmodule

//======================================================================
//======================================================================

//======================================================================
//  MODULE 2 of 3 :  gray2bin
//
//  Converts a Gray-code value back to plain binary.
//
//  Going the other way is not a simple shift-and-XOR, because each
//  binary bit depends on every Gray bit above it:
//
//      bin[MSB] = gray[MSB]
//      bin[i]   = bin[i+1] XOR gray[i]        (working downwards)
//
//  which is the same as saying bin[i] is the XOR of all Gray bits from
//  the MSB down to position i.  The loop below just builds that chain
//  one bit at a time, which is easier to read than a set of reduction
//  XORs and synthesises to exactly the same thing.
//
//  Worked example, 4 bits, gray = 4'b0101:
//
//      bin[3] = gray[3]          = 0
//      bin[2] = bin[3] ^ gray[2] = 0 ^ 1 = 1
//      bin[1] = bin[2] ^ gray[1] = 1 ^ 0 = 1
//      bin[0] = bin[1] ^ gray[0] = 1 ^ 1 = 0
//      bin = 0110 = 6            <- matches the bin2gray example
//
//  Where this is used in the final FIFO:  the read pointer arrives in
//  the write clock domain as a Gray value.  It has to be compared
//  against the write pointer, and that comparison is easiest to reason
//  about in binary -- so we convert it back after it has safely crossed.
//
//  Also purely combinational.  The `always @*` block below describes
//  logic, not registers -- there is no clock, so nothing is stored.
//======================================================================

module gray2bin #(
    parameter WIDTH = 5
)(
    input  wire [WIDTH-1:0] gray,
    output reg  [WIDTH-1:0] bin
);

    integer i;

    always @* begin
        bin[WIDTH-1] = gray[WIDTH-1];
        for (i = WIDTH-2; i >= 0; i = i - 1)
            bin[i] = bin[i+1] ^ gray[i];
    end

endmodule

//======================================================================
//======================================================================

//======================================================================
//  MODULE 3 of 3 :  sync_2ff
//
//  A two-flip-flop synchronizer: the standard way to bring a signal
//  from one clock domain into another.
//
//  ------------------------------------------------------------------
//  WHAT PROBLEM DOES THIS SOLVE?
//
//  A flip-flop needs its input to be stable for a short window before
//  the clock edge (setup time) and after it (hold time).  A signal
//  coming from a different, unrelated clock has no idea where our clock
//  edges are, so sooner or later it will change inside that window.
//
//  When that happens the flop goes METASTABLE: its output sits at an
//  undefined level part-way between 0 and 1 for an unpredictable time
//  before finally settling to one or the other.  You cannot prevent
//  this.  What you can do is give it time to settle before anything
//  else looks at it.
//
//  That is all the second flop is for.  Stage 0 may go metastable; by
//  the time the next clock edge arrives it has almost certainly settled,
//  and stage 1 captures a clean 0 or 1.  "Almost certainly" is literal
//  -- the failure rate is a probability (mean time between failures),
//  not zero, and adding more stages pushes it further out.
//
//  ------------------------------------------------------------------
//  WHAT THIS DOES *NOT* SOLVE
//
//  The second flop guarantees a clean logic level.  It does NOT
//  guarantee a CORRECT value.  If several bits cross together and two
//  of them change on the same cycle, each bit independently resolves to
//  the old or the new value, and the combination can be a number that
//  never existed.
//
//  That is exactly why the FIFO pointers are Gray-coded before they get
//  here: with only one bit changing at a time, "old or new" is the only
//  possible outcome, and both are safe.  Synchronizer and Gray code are
//  a pair -- neither is sufficient alone.  The Stage 2 testbench
//  demonstrates this.
//
//  ------------------------------------------------------------------
//  ASYNC_REG
//
//  The (* ASYNC_REG = "TRUE" *) attribute tells Vivado two things:
//  place these flops close together (ideally in the same slice) so the
//  settling time is not eaten by routing delay, and do not optimise or
//  retime them away.  Without it the tool may scatter the two stages
//  and quietly ruin the MTBF.  Vivado's report_cdc will flag a missing
//  ASYNC_REG, so this one line is worth remembering.
//
//  Icarus Verilog ignores the attribute; Vivado acts on it.
//======================================================================

module sync_2ff #(
    parameter WIDTH = 5
)(
    input  wire             clk,      // the DESTINATION clock
    input  wire             rst_n,    // reset in the destination domain
    input  wire [WIDTH-1:0] din,      // signal from the source domain
    output wire [WIDTH-1:0] dout      // safe to use in this domain
);

    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] sync_stage0;
    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] sync_stage1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_stage0 <= {WIDTH{1'b0}};
            sync_stage1 <= {WIDTH{1'b0}};
        end else begin
            sync_stage0 <= din;           // may go metastable
            sync_stage1 <= sync_stage0;   // settled by now
        end
    end

    assign dout = sync_stage1;

endmodule
