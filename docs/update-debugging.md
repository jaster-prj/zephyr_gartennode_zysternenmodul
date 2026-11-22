# CAN update investigation

Target: cistern node 12 (`0x0c`), `CanGatewayTest`, `can0`, 125 kbit/s.
Baseline: main `ce8af9d` plus these fixes, Zephyr 4.5.0, matching Gartennode
board module and signing key. See [firmware-updates.md](firmware-updates.md)
for build, bootstrap and update instructions.

## Findings and fixes

1. **Signing/header mismatch.** Run 44945523719 reserved a 1024-byte STM32
   header but signed it as 512 bytes. CI now packages Zephyr's signed output
   and checks layout, vector table and signature.
2. **Wrong default swap assumption.** Newer Zephyr defaults to offset swap,
   while this board has an external secondary slot beginning at offset zero
   and a separate scratch partition. The application explicitly defaults to
   scratch swap.
3. **Redundant reception-time erasing.** CANopen's CLEAR command already
   erases the secondary slot. Stream-flash erasing is disabled during reception.
4. **Sparse SWD recovery left stale swap status.** The sparse signed HEX did
   not cover a primary trailer page. Three old status bytes survived near
   slot offset `0x2f6d0`, while the swap-size field had been erased. MCUboot
   interpreted this as an interrupted swap with size `0xffffffff` and faulted
   in `find_last_sector_idx()`. A full padded, confirmed BIN erased the entire
   primary slot and restored normal cold boot. This was a recovery-state
   problem, separate from CAN data reception.
5. **Confirmation/NMT mismatch.** The application boots OPERATIONAL, but the
   upstream control handler rejected the gateway's `0x80` image confirmation
   outside PRE-OPERATIONAL. The application's adapter permits this explicit
   manufacturer extension while preserving upstream restrictions on standard
   STOP/CLEAR/START commands. Images are not automatically confirmed.
6. **Fixed software-version string.** `0x100A` now reports the actual signed
   version without a C terminator. Release EDS metadata comes from the same
   build configuration, allowing the gateway to verify the new version.

## Controlled hardware tests

No SWD reads or halts were performed during these transfers.

| Receiver → uploaded image | Result |
| --- | --- |
| Diagnostic 1.2.1 → 1.2.2, before recovery-state repair | Entire 196608-byte download accepted, flash status zero; failed after upgrade request in bootloader |
| Diagnostic 1.2.1 → diagnostic 1.2.3 | API success, reboot, `0x100A = 1.2.3`, explicit confirmation |
| Diagnostic 1.2.3 → normal 1.3.0 | API success, reboot, version check and confirmation |
| Normal 1.3.0 → normal 1.3.1 | API success, reboot, `0x100A = 1.3.1`, confirmation; production receiver and image |

The three complete captures each show 196608 acknowledged data bytes, zero
duplicate data requests, maximum data-ACK delay below 8 ms, and no boot-up
heartbeat during data reception. A boot-up heartbeat appears after START,
as expected. A duplicate CLEAR request produces a device-state abort before
data reception because the gateway retries after 500 ms; it does not invalidate
the subsequent successful transfer.

Final state: normal 1.3.1 is installed and confirmed. An additional NMT RESET
returned a boot-up heartbeat followed by OPERATIONAL heartbeats; no debugger
was needed. Confirmation state remained START (`0x01`). The fresh bootloader
uses the original signing key and the scratch configuration supplied here.
Rebuilding with `bootloader.conf` and `debug/bootloader.conf` reproduced its
binary exactly (SHA-256
`e725deb6416ad0a126aefff67c84a6e285333f5b449824b640246413b59c7ea1`).
An additional 160 consecutive read-only SDO checks all returned error register
zero. During that traffic the gateway recovered from ERROR-WARNING to
ERROR-ACTIVE without an interface restart or timing change, consistent with
ACK-error counters accumulated while the target was offline during reboot.

## Watchdog and earlier observations

The running 4.5 baseline has watchdog support disabled. A controlled nine-second
debug halt did not trigger a watchdog reset, and reset-cause flags after the
upgrade request indicated software reset, not IWDG/WWDG reset. No spontaneous
reset was observed during the successful downloads.

Earlier sessions observed CAN errors and no-write test failures, but those
sessions also involved debugger interaction and inconsistent recovery state.
They do not establish a sector-erase or wiring defect. Clean transfers on the
same bus subsequently succeeded. Gateway undervoltage was also reported and
should be corrected, but it was not demonstrated to cause the reproduced
firmware update failures.

`/dev/ttyUSB0` received no console bytes at 115200 baud. Optional deferred RTT
diagnostics record reset causes, download progress and callback latency without
skipping writes or disabling sensors. An immediate-logging experiment overflowed
a sensor thread's stack; immediate logging is not retained.

## Gateway timing caveat

MCUboot swap and application startup took about 16.8 seconds on this target.
The existing gateway starts verification after a fixed ten-second delay and
then retries SDO reads. Its timeout margin is narrow. A future gateway change
should wait for a fresh boot-up/heartbeat with a bounded longer timeout rather
than assume a ten-second reboot. No gateway service binary or CAN timing
configuration was changed for the successful tests.

## Backups

Original internal flash is preserved under `build/update-debug-44945523719/`
and `/home/admin/cistern-update-backup-44945523719/`. The saved original primary
image fails imgtool hash validation; it is forensic material, not a verified
recovery image. Preserve it, but do not automatically restore it as known-good
firmware. Signing keys are excluded from all published artifacts and backups.

The final release package, matching bootloader, verified confirmed bootstrap,
manifest and test evidence are retained in `build/update-1.3.1/` and
`/home/admin/cistern-update-1.3.1/` on the gateway.
