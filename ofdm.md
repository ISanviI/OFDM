> iverilog -g2012 -o ofdm_exe \*.v
> Compile using system_verilog

Real and Imaginary - Twiddle Factors in memory are
N=2048 Complex Twiddle factor for k=0to1023 with Q2.14 fixed-point representation
16-bit two's-complement hexadecimal values

User
│
│ bitstream + bitstream_valid
▼
┌──────────────────┐
│ MODULATOR │
│ │
│ bitstream_ready ◄──── user waits here
│ │
│ I_out ───────────────► IFFT input_real
│ Q_out ───────────────► IFFT input_imag
│ symbol_valid ────────► IFFT input_valid
└──────────────────┘
▲
│
│ ifft_ready
│
└──────── IFFT input_ready

                         │
                         ▼
                  ┌─────────────┐
                  │    IFFT     │
                  │             │
                  │ input_ready │
                  │             │
                  │ output_real │
                  │ output_imag │
                  │ output_valid│
                  └─────────────┘
