`timescale 1ns / 1ps
//======================================================================
//  sync_fifo.v   --   Stage 1 of async_fifo_cdc
//
//  A parameterized SINGLE-CLOCK FIFO.
//
//  Why build this first?  It contains everything the asynchronous FIFO
//  contains EXCEPT the clock-domain crossing:  the storage array, the
//  pointer arithmetic, and the full/empty rules.  Getting these right
//  with one clock means that in Stage 3 the only new thing to debug is
//  the crossing itself.
//
//  The one "trick" here is that the pointers are ADDR_WIDTH+1 bits wide
//  while the memory only needs ADDR_WIDTH bits of address.  That extra
//  top bit is what lets us tell FULL apart from EMPTY -- see the notes
//  at the bottom of this file.
//======================================================================

module sync_fifo #(
    parameter DATA_WIDTH = 8,        // bits per entry
    parameter ADDR_WIDTH = 4         // depth = 2**ADDR_WIDTH = 16 entries
)(
    input  wire                  clk,
    input  wire                  rst_n,      // active-low async reset

    // ---- write side ----
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire                  full,

    // ---- read side ----
    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire                  empty
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    // ------------------------------------------------------------------
    // Storage
    // ------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // ------------------------------------------------------------------
    // Pointers.  Note the width: [ADDR_WIDTH:0], i.e. one EXTRA bit.
    // ------------------------------------------------------------------
    reg [ADDR_WIDTH:0] wptr;
    reg [ADDR_WIDTH:0] rptr;

    // The lower bits are the actual memory address.
    wire [ADDR_WIDTH-1:0] waddr = wptr[ADDR_WIDTH-1:0];
    wire [ADDR_WIDTH-1:0] raddr = rptr[ADDR_WIDTH-1:0];

    // ------------------------------------------------------------------
    // Qualified enables: a write only happens if the FIFO is not full,
    // a read only happens if it is not empty.  This is what makes the
    // FIFO safe against misuse -- a write to a full FIFO is simply
    // ignored rather than corrupting the oldest entry.
    // ------------------------------------------------------------------
    wire do_write = wr_en & ~full;
    wire do_read  = rd_en & ~empty;

    // ------------------------------------------------------------------
    // Write port
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (do_write)
            mem[waddr] <= wr_data;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wptr <= {(ADDR_WIDTH+1){1'b0}};
        else if (do_write)
            wptr <= wptr + 1'b1;
    end

    // ------------------------------------------------------------------
    // Read port
    //
    // rd_data is COMBINATIONAL: it always shows the entry at the head of
    // the FIFO, whether or not rd_en is asserted.  Asserting rd_en simply
    // advances the read pointer so that the next entry appears.
    // (This is the same read style the asynchronous FIFO will use in
    //  Stage 3, which is why it is used here.)
    // ------------------------------------------------------------------
    assign rd_data = mem[raddr];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rptr <= {(ADDR_WIDTH+1){1'b0}};
        else if (do_read)
            rptr <= rptr + 1'b1;
    end

    // ------------------------------------------------------------------
    // Status flags   (see explanation below)
    // ------------------------------------------------------------------
    assign empty = (wptr == rptr);

    assign full  = (wptr[ADDR_WIDTH]     != rptr[ADDR_WIDTH]) &&
                   (wptr[ADDR_WIDTH-1:0] == rptr[ADDR_WIDTH-1:0]);

endmodule

//======================================================================
//  WHY THE EXTRA POINTER BIT?
//
//  A 16-entry FIFO needs a 4-bit address (0..15).  But if the pointers
//  were also 4 bits, then "completely empty" and "completely full" would
//  look identical: in both cases wptr == rptr.  You would have no way to
//  tell them apart.
//
//  Making the pointers 5 bits fixes this.  The bottom 4 bits address the
//  memory; the top bit acts as a "lap counter" that toggles every time a
//  pointer wraps past the end of the array.
//
//    EMPTY : the two pointers are identical in every bit.
//            Same address, same lap  ->  reader has caught up to writer.
//
//    FULL  : same address, but DIFFERENT lap bits.
//            The writer has gone exactly one full lap ahead of the
//            reader and is about to overwrite unread data.
//
//  Worked example on a 16-deep FIFO:
//
//    wptr = 5'b0_0000, rptr = 5'b0_0000  ->  equal            -> EMPTY
//    write 16 entries...
//    wptr = 5'b1_0000, rptr = 5'b0_0000  ->  addr 0 == addr 0,
//                                            lap 1 != lap 0   -> FULL
//    read 16 entries...
//    wptr = 5'b1_0000, rptr = 5'b1_0000  ->  equal            -> EMPTY
//
//  Remember this rule.  In Stage 3 it comes back almost unchanged -- the
//  only difference is that the pointers being compared are Gray-coded
//  and have been passed through synchronizers.
//======================================================================
