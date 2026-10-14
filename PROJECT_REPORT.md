# Asynchronous FIFO-Based Dual-Clock Data Transfer Controller

**Design and verification of a CDC-safe FIFO in Verilog**

Srisudha Akella

---

## Abstract

This project implements a parameterized asynchronous FIFO that transfers data
between two independent, unrelated clock domains without loss, duplication or
reordering. The design uses Gray-coded read and write pointers, two-flip-flop
synchronizers, per-domain reset synchronizers, and status flags derived from an
extra pointer lap bit.

The work was built in stages, each independently verified. A skew-modelling
experiment quantifies why Gray coding is required: under identical wire-delay
conditions, a binary pointer crossing produced 26 values the source counter
never held, while the Gray-coded path produced none. The final design was
verified by a self-checking Verilog testbench across five write-to-read clock
ratios, transferring 1,356 words with zero mismatches. The testbench itself was
validated by fault injection into the pointer comparison logic.

---

## 1. Introduction

### 1.1 The problem

A digital system rarely runs on a single clock. A processor core, a memory
interface and a peripheral bus typically each have their own clock, generated
independently and with no fixed phase relationship. Moving data between such
domains is not a matter of connecting wires, because a flip-flop imposes timing
requirements that an unrelated clock cannot be expected to respect.

Every flip-flop requires its data input to be stable for a short window before
the clock edge (*setup time*) and after it (*hold time*). A signal originating
in another clock domain has no knowledge of where the destination clock edges
fall, so it will eventually change inside that window. When it does, the
flip-flop enters a **metastable** state: its output settles at neither a valid
0 nor a valid 1 for an unpredictable interval before eventually resolving.

This cannot be prevented by design. It can only be managed.

### 1.2 Objective

Design and verify a FIFO that allows a writer in one clock domain and a reader
in another to exchange data safely, where "safely" means three specific
guarantees:

1. Every word written is eventually read, exactly once.
2. Words are read in the order they were written.
3. The status flags never permit an unsafe operation — no write into a full
   FIFO, no read from an empty one.

### 1.3 Tools

Verilog (IEEE 1364-2001), Xilinx Vivado for synthesis and timing analysis,
Icarus Verilog and GTKWave for simulation and waveform inspection.

---

## 2. Background

### 2.1 Metastability and the two-flip-flop synchronizer

The standard mitigation for metastability is to give the signal time to settle
before any logic depends on it. A two-flip-flop synchronizer does exactly this:
the first flip-flop may go metastable; by the time the next clock edge arrives
it has, with very high probability, resolved to a valid level, which the second
flip-flop then captures cleanly.

This is a probabilistic guarantee, expressed as a mean time between failures
(MTBF). It is not a proof of correctness. Additional stages push the MTBF
further out at the cost of latency.

### 2.2 What the synchronizer does not solve

A two-flip-flop synchronizer guarantees a clean **logic level**. It does not
guarantee a correct **value**.

When a multi-bit bus crosses a clock boundary, each bit is captured by its own
flip-flop and resolves independently. If two or more bits change on the same
cycle, one may resolve to its old value while another resolves to its new
value. The combination can be a number that never existed in the source domain.

This is the failure mode that Gray coding addresses, and the two mechanisms are
complementary rather than alternative. Neither is sufficient alone.

### 2.3 Gray code

A Gray code is an ordering of binary values in which consecutive entries differ
in exactly one bit position. Conversion from binary is a single operation:

```
gray = bin ^ (bin >> 1)
```

The reverse conversion accumulates an XOR chain from the most significant bit
downwards:

```
bin[MSB] = gray[MSB]
bin[i]   = bin[i+1] ^ gray[i]
```

The relevant property is illustrated by the 7-to-8 transition:

| count | binary | Gray  | Gray bits changed |
|-------|--------|-------|-------------------|
| 6     | 00110  | 00101 | —                 |
| 7     | 00111  | 00100 | 1                 |
| 8     | 01000  | 01100 | 1                 |

In binary, incrementing from 7 to 8 flips four bits simultaneously. In Gray
code it flips one. With only one bit in flight, a sampler observing the bus
mid-transition can only see the old value or the new value — and both are
values the counter genuinely held.

---

## 3. Design methodology

The design was built in four stages, each fully verified before the next began.
This was deliberate: it ensures that when a bug appears, the set of possible
causes is small, and it guarantees a working deliverable at every point.

| Stage | Deliverable | Purpose |
|-------|-------------|---------|
| 1 | Synchronous FIFO | Storage, pointer arithmetic, flag logic — with no CDC |
| 2 | Gray converters, 2-FF synchronizer | CDC primitives, verified in isolation |
| 3 | Asynchronous FIFO | Integration of stages 1 and 2 |
| 4 | Synthesis and timing | Hardware mapping and constraint analysis |

Stage 1 contains everything the final design contains **except** the clock
domain crossing. Establishing it first meant that the only genuinely new
element in Stage 3 was the crossing itself.

---

## 4. Architecture

### 4.1 Block diagram

```
        WRITE CLOCK DOMAIN          |         READ CLOCK DOMAIN
                                    |
   winc --->+-----------+           |          +------------+<--- rinc
            | wptr_full |           |          | rptr_empty |
   wfull <--|           |           |          |            |---> rempty
            +-----+-----+           |          +-----+------+
                  |  wptr (Gray)    |     rptr (Gray) |
                  |                 |                 |
                  |    +--------------------------+   |
                  +--->|   sync_2ff  (w -> r)     |---+
                       +--------------------------+
                  +---[   sync_2ff  (r -> w)     ]<---+
                       +--------------------------+
                  |                 |                 |
             waddr |                |                 | raddr
                   v                |                 v
   wdata --->+---------------------------------------------+
             |            fifo_mem  (dual port)            |---> rdata
             +---------------------------------------------+
                                    |
   arst_n --->[ rst_sync ]          |         [ rst_sync ]<--- arst_n
```

Only the **pointers** cross between domains. The data path does not require
synchronization, for reasons given in Section 4.5.

### 4.2 Module inventory

| Module | Domain | Function |
|--------|--------|----------|
| `async_fifo` | both | Top level; instantiates and connects all submodules |
| `wptr_full` | write | Write pointer, memory write address, full flag |
| `rptr_empty` | read | Read pointer, memory read address, empty flag |
| `fifo_mem` | both | Dual-port storage array |
| `sync_2ff` | destination | Two-flip-flop synchronizer, parameterized width |
| `rst_sync` | one per domain | Reset synchronizer |
| `bin2gray` | — | Combinational binary-to-Gray converter |
| `gray2bin` | — | Combinational Gray-to-binary converter |

Default configuration is 16 entries of 8 bits. Both are parameters.

### 4.3 Pointer width and the lap bit

For a FIFO of depth 2^n, the memory requires an n-bit address. The pointers,
however, are **n+1 bits wide**.

The reason is that with n-bit pointers, "completely empty" and "completely
full" are indistinguishable — in both cases the write and read pointers hold
the same address. The extra most-significant bit acts as a lap counter,
toggling each time a pointer wraps past the end of the array:

- **Empty**: pointers identical in every bit — same address, same lap. The
  reader has caught up with the writer.
- **Full**: same address, different lap bits. The writer is exactly one lap
  ahead and is about to overwrite unread data.

Worked example for a 16-entry FIFO:

```
wptr = 5'b0_0000, rptr = 5'b0_0000   equal                    -> EMPTY
   (16 writes)
wptr = 5'b1_0000, rptr = 5'b0_0000   same address, lap differs -> FULL
   (16 reads)
wptr = 5'b1_0000, rptr = 5'b1_0000   equal                    -> EMPTY
```

### 4.4 Flag generation in the Gray domain

Because the pointers are compared in Gray form, the binary conditions above
must be translated.

**Empty** is unchanged. Gray coding is a bijection, so equal binary values have
equal Gray codes and equality can be tested directly:

```verilog
empty_next = (rgray_next == rq2_wptr);
```

**Full** requires the top **two** Gray bits to differ, with all lower bits
equal. This follows directly from the conversion rather than being a separate
rule. Given `gray[i] = bin[i] ^ bin[i+1]` with the MSB passing through
unchanged:

```
gray[MSB]   = bin[MSB]                  -> flips when the lap bit flips
gray[MSB-1] = bin[MSB] ^ bin[MSB-1]     -> also flips
gray[i]     = bin[i] ^ bin[i+1]         -> unaffected, for i < MSB-1
```

So "binary lap bit differs, all else equal" maps exactly onto "top two Gray
bits differ, all else equal":

```verilog
full_next = (wgray_next == {~wq2_rptr[ADDR_WIDTH:ADDR_WIDTH-1],
                             wq2_rptr[ADDR_WIDTH-2:0]});
```

Both flags are computed from the *next* pointer value and registered. This
keeps the comparison off the critical path, so no long combinational chain sits
between a pointer and a flag output.

### 4.5 Why the data path needs no synchronizer

The memory is written synchronously in the write domain and read
combinationally in the read domain, with no synchronizer on the data. This is
safe, and the argument is worth stating precisely:

The reader may only access a location when `rempty` is deasserted. `rempty`
only clears after the write pointer has propagated through a two-flip-flop
synchronizer, requiring at least two read-clock cycles. By the time the reader
is permitted to look at a location, the data has been stable there for several
cycles. It is never sampled while changing.

Getting the *control* path right is what makes the *data* path safe.

### 4.6 Pessimistic flags

Each domain sees a version of the other pointer that is two or more cycles out
of date. This makes both flags conservative:

- The writer may see full when the reader has already freed a slot, stalling
  slightly early.
- The reader may see empty when the writer has already pushed data, waiting
  slightly longer than necessary.

Both errors are in the safe direction. The dangerous case — believing space
exists when it does not — cannot occur, because stale information about the
opposite pointer can only make a domain more cautious, never less.

### 4.7 Reset strategy

A reset released at an arbitrary moment relative to a clock can allow different
flip-flops to leave reset on different cycles. In a FIFO this is fatal: if the
write pointer exits reset one cycle before the read pointer, the FIFO begins
operation believing it already holds data.

`rst_sync` implements **asynchronous assert, synchronous de-assert**. Assertion
is immediate and requires no clock, so reset works even before clocks are
running. De-assertion passes through two flip-flops so that release occurs on a
clean edge in that domain. One instance is placed in each clock domain, both
driven from a single external asynchronous reset.

---

## 5. Verification

### 5.1 Stage 1 — synchronous FIFO

A self-checking testbench with an independent reference model implemented as a
circular buffer. All stimulus is applied on the negative clock edge so signals
are stable well before the design samples them, eliminating testbench races.

| Test | Result |
|------|--------|
| Reset leaves FIFO empty | Pass |
| `full` asserts after exactly 16 writes | Pass |
| Writes to a full FIFO ignored | Pass |
| All 16 entries read back in order | Pass |
| Reads from an empty FIFO ignored | Pass |
| 20 cycles of simultaneous read and write | Pass |
| 500 randomised cycles | Pass |

**866 checks, 0 errors.**

### 5.2 Stage 2 — CDC primitives and the skew experiment

Exhaustive verification of both converters over all 32 values of a 5-bit
pointer, confirming round-trip identity and that consecutive Gray codes differ
in exactly one bit. The synchronizer was confirmed to have exactly two cycles
of latency.

The principal experiment models a physical fact usually abstracted away in
simulation: **the wires of a bus are not the same length, so its bits do not
arrive simultaneously.**

A 5-bit counter runs at 100 MHz in domain A and is observed from a domain
clocked at approximately 143 MHz. The same counter is sent across twice — once
as raw binary, once Gray-coded — over wires given per-bit delays of 0.10, 0.90,
0.40, 1.30 and 0.60 ns. The delays are identical for both paths, and **both
paths pass through a two-flip-flop synchronizer.**

A value observed on a crossing bus is legal only if it is the counter's current
or immediately previous value; anything else never existed in the source
domain.

| Metric | Binary path | Gray path |
|--------|-------------|-----------|
| Impossible values observed on the wires | **26** | **0** |
| Bad values after synchronization | **3** | **0** |

Representative failures:

```
[135.10 ns] BINARY wires show  2  -- counter is at 4 (was 3)
[135.40 ns] BINARY wires show  6  -- counter is at 4 (was 3)
[269.50 ns] BINARY synced value went BACKWARDS: 15 then 10
```

The counter advanced from 3 to 4. Three of five bits changed, arriving 0.3 ns
apart, and during that interval the bus presented 2 and then 6 — neither of
which the counter ever held. Later, a monotonically increasing counter appeared
to decrease.

The Gray column is zero, and structurally so rather than by good fortune: with
one bit in flight, old-or-new are the only possible observations.

**The binary path in this experiment was synchronized and still failed.** This
is the central result of the project: the synchronizer resolves metastability,
Gray coding ensures the resolved value is a real one. A design needs both.

A control experiment setting all five wire delays equal reduced the binary
failure count to zero, confirming that skew rather than clock asynchrony is the
mechanism.

**67 checks, 0 errors.**

### 5.3 Stage 3 — asynchronous FIFO

With two unrelated clocks, a scoreboard cannot meaningfully track instantaneous
occupancy: "a given instant" is not well defined across asynchronous domains,
and the flags are deliberately pessimistic. Verification therefore targets the
property that matters — *the reader observes exactly the sequence the writer
sent, in order, with nothing lost or duplicated.*

The writer emits an incrementing pattern; the reader maintains an independent
count and checks each arrival against what that count predicts. Any drop,
duplicate or reordering breaks the match immediately.

Directed tests:

| Test | Result |
|------|--------|
| Reset gives `rempty`=1, `wfull`=0 | Pass |
| `wfull` asserts after exactly 16 writes | Pass |
| 20 write attempts while full change nothing | Pass |
| All 16 words read back in order | Pass |
| 20 read attempts while empty change nothing | Pass |

Sustained traffic across five clock ratios:

| Phase | Write clock | Read clock | Words | Result |
|-------|-------------|------------|-------|--------|
| Equal clocks (phase offset) | 10.0 ns | 10.0 ns | 300 | Pass |
| Fast write | 6.0 ns | 14.0 ns | 225 | Pass |
| Slow write | 14.0 ns | 6.0 ns | 215 | Pass |
| Awkward ratio | 7.0 ns | 3.0 ns | 428 | Pass |
| Bursty, random enables | 9.0 ns | 11.0 ns | 188 | Pass |

**1,356 words transferred, 0 mismatches.**

Even for the equal-period case the read clock is given a 1.3 ns phase offset.
Edge-aligned clocks would conceal precisely the behaviour under test.

### 5.4 Validating the testbench

A checker that has never failed has not been shown capable of failing. The full
comparison in `wptr_full` was deliberately corrupted to invert one
most-significant bit instead of two. The testbench reported the failure
immediately:

```
[806000] DATA ERROR: word 0 came out as 0x10, expected 0x00
[816000] DATA ERROR: word 1 came out as 0x11, expected 0x01
```

### 5.5 A testbench defect found during development

An early version of the Stage 3 testbench counted writes with the condition
`run_wr && winc && !wfull`, whereas the design uses `winc && !wfull`. Because
`run_wr` could fall part-way through a cycle while `winc` remained asserted,
the FIFO accepted words the testbench did not count, producing a spurious
mismatch of 300 reads against 299 writes.

The design was correct; the checker was wrong. The lesson is general: **a
scoreboard's trigger condition must match the design's exactly**, or the
verification environment will manufacture bugs that do not exist.

---

## 6. Synthesis and timing analysis

*This section is to be completed after running the flow in Vivado. The
methodology below is fixed; the numbers are to be filled from the reports.*

### 6.1 Constraint strategy

Two independent clocks are declared, and the paths between them are excluded
from timing analysis:

```tcl
create_clock -period 10.000 -name wclk [get_ports wclk]
create_clock -period  7.000 -name rclk [get_ports rclk]
set_clock_groups -asynchronous -group [get_clocks wclk] -group [get_clocks rclk]
```

`set_clock_groups -asynchronous` is required and its consequence must be stated
honestly: once applied, cross-domain paths are **not** timed. Reported WNS and
TNS describe timing *within* each domain. Crossings are verified structurally,
not by slack analysis. Claiming to have "closed timing on the CDC paths" would
misrepresent what the tool did.

### 6.2 Checks to perform

| Check | Command | What it confirms |
|-------|---------|------------------|
| CDC structure | `report_cdc` | Every crossing synchronized; `ASYNC_REG` present; no unsafe multi-bit crossings |
| Utilization | `report_utilization` | LUT/FF/BRAM cost; whether memory inferred as distributed or block RAM |
| Timing | `report_timing_summary` | WNS, TNS, critical path per domain |

### 6.3 Results

| Metric | Write domain | Read domain |
|--------|--------------|-------------|
| Target period | | |
| WNS | | |
| TNS | | |
| Critical path | | |

| Resource | Used |
|----------|------|
| LUTs | |
| Flip-flops | |
| Block RAM | |

`report_cdc` findings: *to be recorded.*

---

## 7. Conclusion

A parameterized asynchronous FIFO was designed and verified, meeting all three
objectives of Section 1.2 across five clock ratios with no data loss,
duplication or reordering.

The project's central technical finding is that Gray coding and synchronization
address two distinct failure mechanisms and that neither substitutes for the
other. This was demonstrated rather than asserted: an experiment holding wire
skew and synchronizer structure constant while varying only the encoding
produced 26 impossible values on the binary path and none on the Gray path.

### 7.1 Limitations

- Metastability is modelled only in its consequences, not in its physics.
  Simulation cannot reproduce the resolution behaviour of a real flip-flop;
  MTBF is a physical property of the silicon and lies outside this analysis.
- Cross-domain paths are excluded from static timing analysis by construction,
  so no slack figure exists for the crossings themselves.
- Verification is directed and constrained-random over a bounded set of clock
  ratios; it is not exhaustive and includes no formal proof.

### 7.2 Future work

- Programmable almost-full and almost-empty thresholds, which real FIFOs
  require so that downstream logic receives advance warning.
- Sticky overflow and underflow error flags for detecting misuse.
- Formal property checking of the pointer comparison logic.
- A standard streaming interface wrapper (for example AXI4-Stream) to make the
  block reusable in a larger system.

---

## 8. File manifest

| File | Contents |
|------|----------|
| `sync_fifo.v` | Stage 1 synchronous FIFO |
| `tb_sync_fifo.v` | Stage 1 testbench |
| `stage2_design.v` | `bin2gray`, `gray2bin`, `sync_2ff` |
| `stage2_tb.v` | Stage 2 testbench including the skew experiment |
| `stage3_design.v` | `async_fifo`, `rst_sync`, `fifo_mem`, `wptr_full`, `rptr_empty` |
| `stage3_tb.v` | Stage 3 testbench |

Simulation:

```
iverilog -o s1.out sync_fifo.v tb_sync_fifo.v            && vvp s1.out
iverilog -o s2.out stage2_design.v stage2_tb.v           && vvp s2.out
iverilog -o s3.out stage2_design.v stage3_design.v stage3_tb.v && vvp s3.out
```

`stage3_design.v` instantiates `bin2gray` and `sync_2ff` from `stage2_design.v`
and must be compiled alongside it.

---

## 9. References

1. C. E. Cummings, "Simulation and Synthesis Techniques for Asynchronous FIFO
   Design," SNUG San Jose, 2002.
2. C. E. Cummings and P. Alfke, "Simulation and Synthesis Techniques for
   Asynchronous FIFO Design with Asynchronous Pointer Comparisons," SNUG San
   Jose, 2002.
3. Xilinx, *Vivado Design Suite User Guide: Design Analysis and Closure
   Techniques* (UG906), section on CDC reporting.
4. IEEE Standard 1364-2001, *Verilog Hardware Description Language*.
