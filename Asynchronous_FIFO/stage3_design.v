`timescale 1ns / 1ps
//======================================================================
//  stage3_design.v   --   Stage 3 of async_fifo_cdc
//
//  THE ASYNCHRONOUS FIFO.
//
//  Modules in this file:
//      async_fifo    top level, ties everything together
//      rst_sync      reset synchronizer (one per clock domain)
//      fifo_mem      the dual-port storage array
//      wptr_full     write pointer + full flag   (write clock domain)
//      rptr_empty    read pointer + empty flag   (read clock domain)
//
//  ---------------------------------------------------------------
//  DEPENDS ON stage2_design.v  --  this file instantiates bin2gray
//  and sync_2ff from Stage 2.  Compile both files together:
//
//      iverilog -o s3.out stage2_design.v stage3_design.v stage3_tb.v
//
//  In Vivado, just have all three files in the project and set
//  stage3_tb as the simulation top.
//  ---------------------------------------------------------------
//
//  HOW IT WORKS, IN ONE PARAGRAPH
//
//  Two pointers, exactly as in the Stage 1 synchronous FIFO, each one
//  bit wider than the address so that a lap bit distinguishes full from
//  empty.  The difference is that each side needs to see the OTHER
//  side's pointer, and that pointer lives in a different clock domain.
//  So each pointer is Gray-coded (one bit changes per increment) and
//  passed through a 2-flip-flop synchronizer before the other side
//  compares against it.  Stage 2 proved why both of those are needed.
//
//  ONE CONSEQUENCE WORTH UNDERSTANDING
//
//  Each side sees a version of the other pointer that is a couple of
//  cycles out of date.  This makes the flags PESSIMISTIC, never wrong:
//
//    - the writer may think the FIFO is full when the reader has
//      already freed a slot;  it stalls slightly too early.
//    - the reader may think it is empty when the writer has already
//      pushed data;  it waits slightly too long.
//
//  Both are safe.  The dangerous direction -- believing there is room
//  when there is not -- cannot happen, because stale information about
//  the other pointer can only ever make your own side more cautious.
//======================================================================


//======================================================================
//  MODULE 1 of 5 :  async_fifo   (top level)
//======================================================================
module async_fifo #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4                 // depth = 2**ADDR_WIDTH
)(
    input  wire                  arst_n,     // one async reset for the whole block

    // ---- write clock domain ----
    input  wire                  wclk,
    input  wire                  winc,       // write request
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire                  wfull,

    // ---- read clock domain ----
    input  wire                  rclk,
    input  wire                  rinc,       // read request
    output wire [DATA_WIDTH-1:0] rdata,
    output wire                  rempty
);

    // Gray-coded pointers, and their synchronized copies in the
    // opposite domain.
    wire [ADDR_WIDTH:0] wptr;        // write pointer, in write domain
    wire [ADDR_WIDTH:0] rptr;        // read  pointer, in read  domain
    wire [ADDR_WIDTH:0] wq2_rptr;    // read  pointer, seen from write domain
    wire [ADDR_WIDTH:0] rq2_wptr;    // write pointer, seen from read  domain

    wire [ADDR_WIDTH-1:0] waddr, raddr;

    // Per-domain resets, both derived from the single async reset.
    wire wrst_n, rrst_n;

    rst_sync u_wrst (.clk(wclk), .arst_n(arst_n), .rst_n(wrst_n));
    rst_sync u_rrst (.clk(rclk), .arst_n(arst_n), .rst_n(rrst_n));

    //------------------------------------------------------------------
    // The two crossings.  Note that only the POINTERS cross, never the
    // data -- and the pointers are Gray-coded, so exactly one bit of
    // each is in flight at any moment.
    //------------------------------------------------------------------
    sync_2ff #(.WIDTH(ADDR_WIDTH+1)) u_sync_r2w (
        .clk(wclk), .rst_n(wrst_n), .din(rptr), .dout(wq2_rptr)
    );

    sync_2ff #(.WIDTH(ADDR_WIDTH+1)) u_sync_w2r (
        .clk(rclk), .rst_n(rrst_n), .din(wptr), .dout(rq2_wptr)
    );

    //------------------------------------------------------------------
    wptr_full #(.ADDR_WIDTH(ADDR_WIDTH)) u_wptr_full (
        .wclk(wclk), .wrst_n(wrst_n), .winc(winc),
        .wq2_rptr(wq2_rptr), .waddr(waddr), .wptr(wptr), .wfull(wfull)
    );

    rptr_empty #(.ADDR_WIDTH(ADDR_WIDTH)) u_rptr_empty (
        .rclk(rclk), .rrst_n(rrst_n), .rinc(rinc),
        .rq2_wptr(rq2_wptr), .raddr(raddr), .rptr(rptr), .rempty(rempty)
    );

    fifo_mem #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_mem (
        .wclk(wclk), .wclken(winc & ~wfull), .waddr(waddr), .wdata(wdata),
        .raddr(raddr), .rdata(rdata)
    );

endmodule


//======================================================================
//  MODULE 2 of 5 :  rst_sync   (reset synchronizer)
//
//  A reset that is released at a random moment relative to the clock
//  can let different flip-flops leave reset on different cycles.  In a
//  FIFO that is fatal: if the write pointer comes out of reset one
//  cycle before the read pointer, the FIFO starts life believing it
//  already contains data.
//
//  The fix is "asynchronous assert, synchronous de-assert":
//
//    ASSERT     -- immediate, no clock needed.  Reset must work even
//                  when the clock is not running.
//    DE-ASSERT  -- passed through two flops, so release happens on a
//                  clean clock edge in this domain.
//
//  One of these is instantiated per clock domain.  This is the module
//  most student projects leave out, and it is a good thing to be able
//  to point at in an interview.
//======================================================================
module rst_sync (
    input  wire clk,
    input  wire arst_n,      // asynchronous, active low
    output wire rst_n        // synchronous release, active low
);

    (* ASYNC_REG = "TRUE" *) reg rst_meta;
    (* ASYNC_REG = "TRUE" *) reg rst_sync_q;

    always @(posedge clk or negedge arst_n) begin
        if (!arst_n) begin
            rst_meta   <= 1'b0;
            rst_sync_q <= 1'b0;
        end else begin
            rst_meta   <= 1'b1;      // a constant 1 walks through
            rst_sync_q <= rst_meta;  // two flops after release
        end
    end

    assign rst_n = rst_sync_q;

endmodule


//======================================================================
//  MODULE 3 of 5 :  fifo_mem   (the storage)
//
//  Written synchronously in the write domain, read combinationally in
//  the read domain.  That sounds alarming -- data crossing domains with
//  no synchronizer at all -- but it is safe, and the reason is worth
//  understanding:
//
//  The reader is only ever allowed to read a location when rempty = 0.
//  rempty only clears after the write pointer has travelled through a
//  2-flop synchronizer, which takes at least two read clocks.  By the
//  time the reader is permitted to look at a location, the data has
//  been sitting there stable for several clock cycles.  It is never
//  read while it is changing.
//
//  This is why the FIFO only needs to synchronize POINTERS.  Getting
//  the control right is what makes the data safe.
//======================================================================
module fifo_mem #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4
)(
    input  wire                  wclk,
    input  wire                  wclken,
    input  wire [ADDR_WIDTH-1:0] waddr,
    input  wire [DATA_WIDTH-1:0] wdata,
    input  wire [ADDR_WIDTH-1:0] raddr,
    output wire [DATA_WIDTH-1:0] rdata
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    always @(posedge wclk) begin
        if (wclken)
            mem[waddr] <= wdata;
    end

    assign rdata = mem[raddr];

endmodule


//======================================================================
//  MODULE 4 of 5 :  wptr_full   (write pointer and full flag)
//
//  Lives entirely in the write clock domain.
//
//  THE FULL CONDITION IN GRAY CODE
//
//  In Stage 1, with binary pointers, full meant:
//        top bit differs, all other bits equal.
//
//  The same condition in Gray code becomes:
//        TOP TWO bits differ, all other bits equal.
//
//  That is not an extra rule to memorise -- it falls straight out of
//  the conversion.  Recall gray[i] = bin[i] ^ bin[i+1], with the top
//  bit passing through unchanged:
//
//     gray[MSB]   = bin[MSB]                  -> flips when bin[MSB] flips
//     gray[MSB-1] = bin[MSB] ^ bin[MSB-1]     -> also flips
//     gray[i]     = bin[i] ^ bin[i+1]         -> unaffected, for i < MSB-1
//
//  So "binary top bit differs, rest equal" maps exactly onto "Gray top
//  two bits differ, rest equal".  Write it out for a 5-bit pointer and
//  you will see it.
//
//  The flag is computed from the NEXT pointer value and registered.
//  That keeps the comparison off the critical path -- there is no long
//  chain of combinational logic between a pointer and the flag output.
//======================================================================
module wptr_full #(
    parameter ADDR_WIDTH = 4
)(
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  winc,
    input  wire [ADDR_WIDTH:0]   wq2_rptr,    // read pointer, synchronized in
    output wire [ADDR_WIDTH-1:0] waddr,
    output reg  [ADDR_WIDTH:0]   wptr,        // Gray coded
    output reg                   wfull
);

    reg  [ADDR_WIDTH:0] wbin;                 // same pointer in binary
    wire [ADDR_WIDTH:0] wbin_next;
    wire [ADDR_WIDTH:0] wgray_next;

    // advance only when a write actually happens
    assign wbin_next = wbin + (winc & ~wfull);

    // reuse the Stage 2 converter
    bin2gray #(.WIDTH(ADDR_WIDTH+1)) u_b2g (.bin(wbin_next), .gray(wgray_next));

    // the memory address is the binary pointer without its lap bit
    assign waddr = wbin[ADDR_WIDTH-1:0];

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wbin <= {(ADDR_WIDTH+1){1'b0}};
            wptr <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            wbin <= wbin_next;
            wptr <= wgray_next;
        end
    end

    // full: top two Gray bits differ, everything below matches
    wire full_next = (wgray_next == {~wq2_rptr[ADDR_WIDTH:ADDR_WIDTH-1],
                                      wq2_rptr[ADDR_WIDTH-2:0]});

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) wfull <= 1'b0;
        else         wfull <= full_next;
    end

endmodule


//======================================================================
//  MODULE 5 of 5 :  rptr_empty   (read pointer and empty flag)
//
//  Lives entirely in the read clock domain.  Simpler than the write
//  side, because empty is just equality:
//
//        empty  =  my read pointer has caught up with the write pointer
//
//  and Gray code does not disturb equality -- if two binary values are
//  the same, their Gray codes are the same.  So the comparison can be
//  done directly on the Gray values with no conversion.
//======================================================================
module rptr_empty #(
    parameter ADDR_WIDTH = 4
)(
    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  rinc,
    input  wire [ADDR_WIDTH:0]   rq2_wptr,    // write pointer, synchronized in
    output wire [ADDR_WIDTH-1:0] raddr,
    output reg  [ADDR_WIDTH:0]   rptr,        // Gray coded
    output reg                   rempty
);

    reg  [ADDR_WIDTH:0] rbin;
    wire [ADDR_WIDTH:0] rbin_next;
    wire [ADDR_WIDTH:0] rgray_next;

    assign rbin_next = rbin + (rinc & ~rempty);

    bin2gray #(.WIDTH(ADDR_WIDTH+1)) u_b2g (.bin(rbin_next), .gray(rgray_next));

    assign raddr = rbin[ADDR_WIDTH-1:0];

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rbin <= {(ADDR_WIDTH+1){1'b0}};
            rptr <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            rbin <= rbin_next;
            rptr <= rgray_next;
        end
    end

    // empty: the pointers match exactly
    wire empty_next = (rgray_next == rq2_wptr);

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) rempty <= 1'b1;     // note: resets to EMPTY, not to 0
        else         rempty <= empty_next;
    end

endmodule
