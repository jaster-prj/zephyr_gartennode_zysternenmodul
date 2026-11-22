# Gartennode firmware updates

## Image layout and version

Build with the Gartennode board module and the matching private signing key at
`bootloader/mcuboot/gartennode.pem` in the west workspace. Do not commit the key.

The application uses MCUboot **swap using scratch**:

* Bootloader: internal flash `0x08000000`, 64 KiB.
* Primary image: internal flash `0x08010000`, 192 KiB.
* Secondary image: external SPI flash offset `0`, 192 KiB.
* Scratch: external SPI flash offset `0x30000`, 192 KiB.

`CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` is the release version (default `1.3.1`).
Override it in an extra configuration fragment for the next release. It controls
the signed header and the running application's `0x100A` VISIBLE_STRING. The
release EDS is generated from the same configuration, not the generated object
dictionary's older fixed string.

From the west workspace:

```sh
west build -b gartennode application -d build/cistern
python application/scripts/check_firmware.py \
  build/cistern/zephyr/zephyr.signed.bin build/cistern/zephyr/.config
python application/scripts/prepare_eds.py \
  build/cistern/zephyr/.config application/objdict/objdict.eds objdict.eds
```

Publish `zephyr.signed.bin`; do not sign it again with a hard-coded header size.
Zephyr 4.5 uses a 1024-byte STM32 header, whereas the earlier build used 512 bytes.

## SWD bootstrap/recovery

**Use a full padded BIN for the primary slot.** A sparse HEX only causes the
programmer to erase pages represented in that file. Old swap-status bytes in
an unrepresented trailer page can survive, while the swap-size field in the
last page is erased. MCUboot then sees inconsistent interrupted-swap state.

For a verified initial baseline, generate the confirmed variant:

```sh
west build -d build/cistern -- -DCONFIG_MCUBOOT_GENERATE_CONFIRMED_IMAGE=y
STM32_Programmer_CLI -c port=SWD mode=UR \
  -d build/cistern/zephyr/zephyr.signed.confirmed.bin 0x08010000 -v \
  -rst -run
```

The padded BIN must be exactly `0x30000` bytes. This erases/programs the entire
primary slot, including its status area, without mass-erasing the bootloader
or external settings. Preserve a complete backup before recovery. If an old
swap is pending in external flash, resolve or clear that pending update as
part of installing a consistent baseline; do not resume an interrupted swap
against an unrelated replacement primary image.

The matching bootloader configuration is `bootloader.conf`:

```sh
west build -b gartennode bootloader/mcuboot/boot/zephyr \
  -d build/cistern-boot \
  -- -DEXTRA_CONF_FILE="$PWD/application/bootloader.conf"
```

Add `application/debug/bootloader.conf` as a second extra fragment for RTT
bootloader logs.

## CAN update

Upload the signed BIN using the version reported by the new image:

```sh
curl --cacert CA.pem \
  -H 'Content-Type: application/octet-stream' \
  --data-binary @build/cistern/zephyr/zephyr.signed.bin \
  'https://cangatewaytest.ll-tj-home/canopenrest/api/v1/flash?node=0xC&version=1.3.1'
```

The response is a job ID. Read its final status through
`GET /canopenrest/api/v1/flash?id=<id>`. A successful transfer alone is not a
successful update: require reboot, the expected `0x100A` version, and explicit
image confirmation.

STOP, CLEAR and START retain the upstream PRE-OPERATIONAL restriction.
Zephyr's explicit confirmation command (`0x1F51:1 = 0x80`) is also accepted
while OPERATIONAL, because this application's newly booted image enters that
state before the gateway verifies and confirms it. Confirmation is **not**
automatic; unconfirmed test images retain MCUboot's rollback protection.

## Diagnostics

Add `application/debug/update.conf` to `EXTRA_CONF_FILE` to record reset causes,
download byte counts, callback latency and control results. It uses deferred
RTT logging and suppresses missing-sensor log noise without disabling sensors.

Do not halt or read the target through SWD during a timed CAN transfer. Read the
`cistern_update_diagnostics` counters and RTT buffer only after the test ends.
Capture SDO traffic and boot-up heartbeats (`0x70C`, data `00`) independently.
The normal application and supplied bootloader configuration do not enable a
watchdog; any future watchdog addition must service it during update operations.
