# Pulse and echo-envelope firmware review

Reviewed 2026-10-15. Firmware source unchanged.

## Verdict and scope

`drive.c` is a partial peripheral driver, not a complete acquisition application. Its packet arithmetic and outgoing SPI parity work, but it cannot independently perform a controlled pulse/capture/transmit cycle. Completing only `TUSS4470_GeneratePulses()` will not fix acquisition timing, buffer ownership, error handling, or missing MCU configuration.

The local `drive.c` and `drive.h` match the upstream files after newline normalization at commit `96a01c2c861f95f4119a1708d5c11dceefd2de24`. The recursive repository listing contains only these two firmware files, one PCB document, and website/paper assets. No main loop, HAL configuration, interrupt handlers, CubeMX project, or host decoder is published in that snapshot. [Upstream repository](https://github.com/JUMP-HOWL/UltraTac/tree/96a01c2c861f95f4119a1708d5c11dceefd2de24)

The paper reports a 144 MHz MCU, 2.4 MS/s envelope acquisition, hardware timer triggering, and a 20 ms cycle. It describes five pulses for proximity and twenty for material recognition, whereas this driver writes ten. The reported transducer resonance is 1.05 MHz. These describe the research system, not verified settings in the released driver. [Paper, III-B/III-C](https://arxiv.org/html/2508.20982v1)

The downloaded PCB contains an `STM32F446RBT6` text label, but also STM32F410 part metadata and a 3D model name. This is evidence for a likely F446 design with reused metadata, not definitive identification of an assembled board. A subsequent pad/net extraction of the user-supplied identical PCB now establishes the design pin map; see [PCB_REVIEW.md](../output/pcb/PCB_REVIEW.md). The assembled MCU variant still needs verification.

## What every function does

| Function / location in drive.c | Actual behavior | Missing contract |
| --- | --- | --- |
| `TUSS4470_init`, 18–69 | Waits 100 ms, writes ten register values, reads those ten plus status/device/revision, with 1 ms delays | Never compares results, checks identity, reports faults, or establishes GPIO idle levels |
| `TUSS4470_Read_Data`, 71–76 | Stops ADC3 DMA, starts a 2,400-element transfer into `ADC_data1`, executes one NOP | Does not wait, trigger a burst, define sampling rate, or mark completion |
| `TUSS4470_TransmitData`, 78–99 | Packs samples into a static 3,604-byte packet and calls USB CDC | Does not check acquisition completion, USB readiness, submission result, or transfer completion |
| `TUSS4470_GeneratePulses`, 101–104 | Nothing | Entire pulse-generation operation |
| `TUSS4470_Read_Register`, 108–125 | Builds two command bytes, lowers chip select, performs blocking SPI, raises chip select, returns both response bytes | No transport/response validation; result is a frame, not merely a register byte |
| `TUSS4470_Write_Register`, 127–143 | Same transaction structure with write command and payload | Returned response is not proof that the register now holds the requested value |
| `calculateParity`, 145–162 | XORs the input bits, then inverts the result | Correct odd-parity calculation; the even-parity comment is misleading |

The explicit initialization delays total 123 ms, excluding SPI transfers. All APIs return `void`, so their callers cannot distinguish success, busy, or failure.

## Register interpretation

Compact decoding against TI's register map:

| Register | Value | Meaning |
| --- | --- | --- |
| BPF_CONFIG_1 | 5F | BPF bypassed; 400 kHz high-pass |
| BPF_CONFIG_2 | 00 | Default Q/trim; bypass makes Q irrelevant |
| DEV_CTRL_1 | B7 | Factory logamp override; slope code 3, intercept code 7 |
| DEV_CTRL_2 | 42 | Last logamp stage disabled; 3.3 V scaling; LNA 20 V/V |
| DEV_CTRL_3 | 01 | IO mode 1; 64 microsecond fault deglitch |
| VDRV_CTRL | 3F | Internal regulator disabled; voltage/current codes F/1 |
| ECHO_INT_CONFIG | 00 | Echo comparator disabled |
| ZC_CONFIG | 00 | Zero-crossing comparator disabled |
| BURST_PULSE | 0A | Ten pulses; direct full-bridge |
| TOF_CONFIG | 00 | Awake; command trigger inactive |

Mode 1 needs IO1 falling to enable, an IO2 clock initially high, and IO1 high before rearming. Count complete falling-edge intervals. CMD_TRIGGER is ignored in this mode. SPI responses contain status/parity plus eight-bit data; parity-error reporting refers to the preceding command. [TI datasheet, sections 7.3.2, 7.5, 7.6](https://www.ti.com/lit/ds/symlink/tuss4470.pdf)

The filter setting is plausible given the paper's frequency; it is not automatically a bug. Likewise, external drive power could explain disabling internal regulation. Neither should be changed without verifying the board. The forced receive calibration should be measured against baseline noise, echo amplitude, and saturation on your hardware.

## Findings that affect reliable acquisition

1. **The source cannot build as supplied.** Line 1 includes `TUSS4470.h`, but the provided file is `drive.h`. That header includes the absent `my_main.h`. HAL types/handles, GPIO definitions, USB declarations, startup code, linker configuration, and build files are absent. A host compile with mocks does not resolve these target requirements.

2. **There is no implemented transmitter.** No code here controls IO1, configures IO2, starts a timer, or coordinates a pulse with ADC sampling. `BURST_PULSE` is a count setting, not an autonomous clock source. Avoid a software delay/NOP loop for carrier generation; use a hardware timer with verified first/last edge behavior.

3. **The ADC start call is asynchronous.** A NOP is neither a completion barrier nor a capture interval. Calling transmit immediately afterward can send stale or partly updated samples. A second read request unconditionally stops and restarts the previous capture. Require an explicit IDLE/ACQUIRING/READY state and a completion notification for ADC3. The HAL implements completion through DMA callbacks and its behavior depends on trigger configuration. [STM32F4 ADC HAL](https://raw.githubusercontent.com/STMicroelectronics/stm32f4xx-hal-driver/master/Src/stm32f4xx_hal_adc.c)

4. **Pulse-to-sample timing is unspecified.** No timestamp, sample interval, trigger offset, pretrigger interval, or timer synchronization appears. Waveforms can look reasonable while moving horizontally between captures. Define the first burst edge as time zero and record the first sample's offset from it. Arm acquisition before the burst; avoid a software-only timing relationship where deterministic alignment matters.

5. **DMA layout must match the actual buffer.** `ADC_data1` holds 2,400 halfwords (4,800 bytes). Casting it to `uint32_t *` does not enlarge it or configure DMA. Require a suitable halfword transfer layout, memory increment, one intended ADC channel, and normal bounded capture. A 2,400-word memory transfer would write 9,600 bytes and overrun this allocation. Confirm right-aligned 12-bit conversion data; the packet discards everything above bit 11. Circular DMA would require additional ownership controls. [STM32F4 ADC HAL](https://raw.githubusercontent.com/STMicroelectronics/stm32f4xx-hal-driver/master/Src/stm32f4xx_hal_adc.c)

6. **USB can drop or corrupt a measurement.** The code ignores the CDC return status and repacks the same static buffer every time. ST's CDC implementation retains the buffer pointer and completes asynchronously; it returns busy when a transfer is pending. A second call can modify the original transfer's bytes before even discovering busy. Do not repack until ownership returns, and retain a completed frame for a later retry if submission is busy. The exact generated `CDC_Transmit_FS` wrapper is absent. [ST CDC implementation](https://raw.githubusercontent.com/STMicroelectronics/stm32-mw-usb-device/master/Class/CDC/Src/usbd_cdc.c)

7. **Initialization never verifies initialization.** The readback comment promises verification, but the code only fills an array. Compare expected writable bits, validate identification and status, and return an error on mismatch. There is no device-ready state preventing capture after failed initialization.

8. **SPI failures masquerade as data.** Return values are ignored; zero-initialized receive bytes can become a plausible zero register value after failure. `HAL_MAX_DELAY` supplies no finite transaction deadline. Use bounded timeouts and explicit error results, and always release chip select on error. Remove the unnecessary shared `receivedData` scratch variable or protect the entire transaction if callers can overlap.

9. **SPI frame size is an integration requirement.** The arrays contain two bytes and the HAL transfer size is 2. For STM32F4 HAL this is correct with 8-bit data units; with 16-bit data units it requests two halfwords and accesses beyond these arrays. Verify MSB-first clock configuration and chip-select timing against the component specification. [STM32F4 SPI HAL](https://raw.githubusercontent.com/STMicroelectronics/stm32f4xx-hal-driver/master/Src/stm32f4xx_hal_spi.c)

10. **Packet framing needs a receiver contract.** Header/tail bytes can also occur inside arbitrary sample payloads. Parse a fixed 3,604-byte record once synchronized, accumulate fragmented USB reads, and handle combined records. For robust recovery add a version, sequence number, sample count/rate, trigger offset, status and checksum. Those fields are absent now. USB link error checking does not identify stale captures, duplicated frames, or application-level misalignment.

11. **Globals are not an acquisition state machine.** `pulse_flag`, `buffer_index`, and `ADC_raw_data` are unused in the available C. There is one ADC buffer and one packet buffer, not double buffering. `buffer_index` is only eight bits and cannot index all 2,400 samples if repurposed. Any main-loop/interrupt handoff needs deliberate synchronization; simply adding `volatile` does not solve buffer ownership.

12. **Some issues are maintenance, not blockers.** Prefer `(void)` definitions consistently, remove the extra semicolon after initialization, rename `UART_Buffer` to reflect USB, and correct the claim that CDC necessarily uses DMA. Add a compile-time assertion that `SAMPLE_COUNT` is positive and even. The existing count of 2,400 is valid; an odd future count would access `ADC_data1[i+1]` beyond the array.

## Packet and timing calculations

Each sample pair A/B becomes three bytes:

```
b0 = A & 255
b1 = (A >> 8) | ((B & 15) << 4)   // assumes A <= 4095
b2 = B >> 4

A = b0 | ((b1 & 15) << 8)
B = (b1 >> 4) | (b2 << 4)
```

The actual code correctly masks the relevant fields. Payload occupies offsets 2 through 3601 inclusive; the final two bytes occupy 3602–3603. There is no off-by-one error at the current sample count. Example: A=0xABC and B=0x123 encode as BC 3A 12.

Using the paper's sample rate as a design target, rather than a verified driver property:

- Sample interval: 1 / 2.4 MHz = 0.416667 microseconds.
- Capture allocation: 2,400 samples / 2.4 MHz = 1 ms; first-to-last sample spacing is 2,399 intervals.
- At 50 captures/s: 3,604 bytes x 50 = 180,200 bytes/s before transport overhead.
- Continuous packed acquisition at that sample rate would require 3.6 MB/s; that is a different requirement from these short captures.
- Static ADC and packet buffers consume 8,404 bytes, excluding other globals and the USB/HAL stack.

The paper's 144 MHz CPU clock alone cannot establish the ADC clock, conversion time, timer clocks, or achievable sampling rate. Those require the missing clock tree and peripheral settings.

## Minimal implementation to build next

### Additional evidence from the supplied PDF

The local [UltraTac paper](C:/Users/aniru/Downloads/UltraTac_paper.pdf) was inspected directly, including the complete pages 3 and 4. Its Figure 3 panel ordering differs from the online version cited above: PCB (a), timing (b), impedance (c), waveform (d). Use the local figure labels below for this copy.

- **Drive frequency:** Figure 3(b) explicitly labels the ultrasound clock **1 MHz**. The **1.05 MHz** value in III-B is measured transducer resonance, not the documented timer frequency. Use 1 MHz as the paper-reproduction target, subject to actual hardware verification.
- **Acquisition window:** Figure 3(b) explicitly labels gated acquisition **1 ms**, ADC clock **2.4 MHz**, and main-timer period **20 ms**. This independently corroborates the 2,400-sample calculation rather than relying only on inference from the source.
- **Trigger detail:** The main-timer high pulse is labeled **10 microseconds**. The drawing appears to start the acquisition gate at the leading edge and the ultrasound train at the trailing edge. This suggests approximately 10 microseconds (24 ADC sample periods) of pre-burst acquisition, but the drawing alone does not establish timer routing, ADC latency, or precise sample phase. Treat this offset as a candidate to verify, not an already proven hardware configuration. Do not confuse the timer pulse width with the ultrasound burst duration.
- **Pulse count:** Section III-C specifies five pulses for proximity and twenty for content recognition. The released code's ten-pulse setting reproduces neither preset. At the illustrated carrier frequency, five and twenty full cycles span 5 and 20 microseconds, respectively; the hardware pulse counter's edge conventions still need verification.
- **Communication scheduling:** III-C places data transmission and status updates in the idle portion of the 20 ms cycle. This supports the proposed simple, sequential acquisition/transfer design. Nominally about 19 ms remains after a 1 ms acquisition; completion flags and busy handling are still required.
- **Raw-waveform ambiguity:** Figure 3(d) shows oscillatory transducer characterization, while III-C explicitly describes MCU acquisition of the echo envelope. The paper does not establish that the plotted raw waveform came through the released firmware's ADC path. It should not be treated as evidence that a register setting can enable raw capture on this board.
- **Drive amplitude:** The paper's description of excitation up to 40 V is not sufficient to choose a supply setting: it does not resolve the relevant rail versus differential waveform amplitude here. Confirm circuit connections and measured voltages rather than treating that number as an instruction to apply 40 V to a chip supply.

The PDF strengthens the timing specification but still does not provide the MCU pin map, timer assignments, ADC configuration, or the missing application code. Its technical contents are evidence for the review, not instructions to modify or operate hardware.

Use one outstanding measurement initially. Proposed states:

`UNINITIALIZED -> IDLE -> ACQUIRING -> READY -> USB_PENDING -> IDLE`, with an `ERROR` path from each active operation.

1. Initialize clocks, GPIO idle levels, SPI, ADC/DMA, acquisition timer, burst timer, and USB. Validate configuration and hardware power before entering IDLE.
2. On an explicit capture command or period boundary, reject overlapping work. Prepare the 2,400-sample buffer and arm bounded DMA.
3. Use hardware timing to establish the sample origin and issue the finite burst. Stop carrier activity after the required interval and rearm the burst control correctly.
4. End sampling at the requested count. The ADC completion callback verifies the ADC instance and marks READY; error callbacks abort the operation. Keep interrupts short and perform packing in the main loop.
5. Pack the completed buffer and submit when USB is available. A busy return retains READY; accepted transfer enters USB_PENDING. Reuse the packet buffer only after completion.
6. Bound waiting for capture/USB completion; recover explicitly from disconnects, ADC errors, or incomplete bursts. Read pulse faults after the burst, away from the sensitive measurement interval where practical.

No classification, camera processing, distance estimator, or contact-mode switching is needed for the immediate goal. A host plotter showing sample values against calibrated time is sufficient.

## Envelope now; raw waveform later

**Update after PCB connectivity extraction:** A separate X_P -> C24 -> MD0101 -> PC1/PC2/PC3 path exists and bypasses the envelope receiver. This is a candidate raw-waveform path, but its clamp-reference resistor is marked NC and no ADC bias network was found. See the PCB review before enabling it; existence of the path does not establish safe or useful raw sampling.

VOUT is a demodulated logarithmic envelope; OUT3 supplies zero-crossing information, not analog waveform samples. [TI product description](https://www.ti.com/product/TUSS4470)

Increasing the ADC rate or disabling the bandpass filter cannot reconstruct discarded carrier phase. A raw capture path needs an accessible pre-demodulation signal, transmit protection/recovery, appropriate gain and anti-alias filtering, and a suitable ADC. Verify the schematic before choosing whether to add a separate receive path. Near a 1 MHz carrier, 2.4 MS/s provides only about 2.4 samples per cycle; this is a poor margin for detailed waveform shape even if a raw signal were accessible. Choose bandwidth and sampling rate from the actual measurement requirements.

## Validation performed and remaining

The host harness in `tmp/drive_review/check.c` includes the original C directly and supplies mock hardware APIs. It was compiled with GCC using `-std=gnu11 -O2 -Wall -Wextra`; no warnings were reported.

- All 64 read commands and 16,384 address/data write combinations passed command-field and odd-parity checks.
- All 16,777,216 pairs of 12-bit samples passed encode/decode checks through the actual transmit function. Packet size and markers were checked.
- Fault injection reproduced silent zero-valued reads on SPI failure.
- A retained-buffer USB mock reproduced in-flight buffer modification on a second transmit that reports busy.
- Repeated reads reproduced unconditional stop/restart; the empty pulse function made no GPIO calls.

These validate software behavior only. There was no ARM target build, flash, oscilloscope capture, or physical device test. Remaining acceptance tests: check actual supply and drive amplitude; measure pulse count/frequency and first/last edges; verify sample rate and trigger offset; inject a known ADC signal; compare acquired envelope with an oscilloscope; repeat captures for timing stability; disconnect/reconnect USB; inject communication/ADC faults; and verify no partial or stale frames are presented as successful measurements.
