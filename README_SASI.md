# SASI initiator support (branch `sasi-initiator`)

This branch of the ZuluSCSI firmware adds **SASI initiator mode**: imaging hard disks
behind early SASI controllers such as the **Xebec S1410 / S1410A** and the
**Western Digital WD1002-SHD** (ST-506/ST-412 MFM drives) to an image file on the SD card.

It is maintained by Mr. MikroMikko for the preservation of Nokia MikroMikko computers.
It is not an official ZuluSCSI release and is not affiliated with Rabbit Hole Computing.
ZuluSCSI is a trademark of Rabbit Hole Computing. The firmware is licensed under the
GNU General Public License version 3, like the upstream project. Upstream is welcome
to take any part of this work.

## Status

Tested on a **ZuluSCSI Blaster 2026c** (RP2350, `ZuluSCSI_Blaster` build) in initiator
mode (DIP switches: Initiator on, termination on):

| Machine | Controller | Drive | Geometry | Result |
|---|---|---|---|---|
| Nokia MikroMikko 1 M7 | Xebec (S1410A compatible) | Seagate ST-406 | 306 × 2 × 32 × 256 | complete, ~5 min, 4 identical reads |
| Nokia MikroMikko 2 | Xebec | Seagate ST-406 | 306 × 2 × 17 × 512 | complete, ~76 s, identical to an independent raw MFM capture |
| Nokia MikroMikko 2A | Western Digital WD1002-SHD (WD1010) | NEC D5126 | 615 × 4 × 17 × 512 | complete, 2 reads identical; cylinders 612–614 unformatted (skipped, zeros) |

The ZuluSCSI Compact RP2040 Rev 2022a could not be used as an initiator (hardware level).

## Configuration (`zuluscsi.ini`)

```ini
[SCSI]
InitiatorSASI = 1                  ; enable SASI mode
InitiatorID = 7
InitiatorSASICylinders = 306
InitiatorSASIHeads = 2
InitiatorSASISectorsPerTrack = 32  ; 17 for 512-byte sectors
InitiatorSASIBlockSize = 256       ; must match the controller's sector size jumper
InitiatorSASIInitParams = 1        ; send Xebec INITIALIZE DRIVE CHARACTERISTICS (0x0C)
InitiatorSASIReducedWriteCyl = 128
InitiatorSASIPrecompCyl = 128
InitiatorSASIEccBurst = 11
InitiatorSASISelectInitiatorBit = 0
InitiatorAllowWrite = 0            ; default: never write to the source drive
InitiatorSASISkipTrackOnSeekError = 0 ; 1: seek error 0x15 skips the whole track (WD1002)
```

**Western Digital WD1002-SHD** (WD1010 based; OEM manual 79-000004): the same 8-byte
parameter block is sent with command 0x0C (SET PARAMETERS). Without it the controller
uses its reset default of 153 cylinders / 4 heads. Settings used for the NEC D5126:

```ini
InitiatorSASICylinders = 615
InitiatorSASIHeads = 4
InitiatorSASISectorsPerTrack = 17
InitiatorSASIBlockSize = 512
InitiatorSASIPrecompCyl = 64
InitiatorSASIEccBurst = 5          ; ECC burst length recommended by the WD manual
InitiatorSASISkipTrackOnSeekError = 1
```

## What the SASI mode does

- Uses the SASI command set: TEST DRIVE READY, INITIALIZE DRIVE CHARACTERISTICS (0x0C),
  RECALIBRATE, READ(6), 4-byte REQUEST SENSE. No INQUIRY / READ CAPACITY / START STOP UNIT;
  the capacity comes from the .ini geometry. No message phases (no IDENTIFY).
- **Controller pauses:** the Xebec pauses inside the data phase while it seeks to an
  alternate track or re-reads a sector. In SASI mode the accelerated read does not abort
  on phase line changes while REQ is inactive; an 8 s timeout ends a real stall.
- **Bus reset loses the geometry:** after a reset the Xebec falls back to its default
  drive characteristics and silently reads wrong sectors. After every failure the drive
  is brought up again (TEST DRIVE READY, 0x0C, RECALIBRATE) before the next read; imaging
  stops rather than reading with the wrong geometry.
- **Alternate tracks (S1410A):** error 0x1C (direct access to an alternate track) and
  0x19 (bad track flag) skip the whole track without retries; the image keeps zeros there.
  The data of a bad track is read through the controller at its logical position.
- **Seek error / ID not found (0x15), optional:** with `InitiatorSASISkipTrackOnSeekError = 1`
  the whole track is skipped (zeros) instead of retrying every sector. On WD1010-based
  controllers 0x15 means that no sector ID was found, e.g. on cylinders that were never
  formatted (the NEC D5126 above was formatted to 612 of its 615 cylinders).
- **Correctable ECC errors (0x18)** are accepted, the controller has corrected the data.
- **Image file fixes (all modes):** skipped sectors are written as zeros (SdFat cannot seek
  past the end of a file on FAT32, and a failed transfer may leave stale buffer data);
  the image is always padded to its full size.

## Write protection

`InitiatorAllowWrite = 0` (the default, also without an SD card or .ini) allows only
commands that do not write to the medium (TEST UNIT READY, REZERO, REQUEST SENSE, READ,
SASI 0x0C, INQUIRY, MODE SENSE, START STOP UNIT, READ CAPACITY). Everything else is
refused and logged, including writes from the USB MSC initiator mode. Writing to SASI
drives is not implemented.
