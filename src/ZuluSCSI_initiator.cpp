/**
 * ZuluSCSI™ - Copyright (c) 2022-2026 Rabbit Hole Computing™
 * BlueSCSI - Copyright (c) 2026 Eric Helgeson, Androda*
 *
 * ZuluSCSI™ firmware is licensed under the GPL version 3 or any later version. 
 *
 * https://www.gnu.org/licenses/gpl-3.0.html
 * ----
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. 
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/


/*
 * Main program for initiator mode.
 */
#include "ZuluSCSI_config.h"
#include "ZuluSCSI_log.h"
#include "ZuluSCSI_log_trace.h"
#include "ZuluSCSI_initiator.h"
#include "ZuluSCSI_msc_initiator.h"
#include "ZuluSCSI_msc.h"
#include <ZuluSCSI_platform.h>
#include <minIni.h>
#include "SdFat.h"
#include "vhd_support.h"
#include "ui.h"
#include "ZuluSCSI_disk.h"
#include <numeric>

#include <scsi2sd.h>
extern "C" {
#include <scsi.h>
}

#ifndef PLATFORM_HAS_INITIATOR_MODE

void scsiInitiatorInit()
{
}

void scsiInitiatorMainLoop()
{
}

int scsiInitiatorRunCommand(const uint8_t *command, size_t cmdlen,
                            uint8_t *bufIn, size_t bufInLen,
                            const uint8_t *bufOut, size_t bufOutLen)
{
    return -1;
}

bool scsiInitiatorReadCapacity(int target_id, uint32_t *sectorcount, uint32_t *sectorsize)
{
    return false;
}

#else

// From ZuluSCSI.cpp
extern bool g_sdcard_present;

/*************************************
 * High level initiator mode logic   *
 *************************************/

// Not in the SCSI_MESSAGE enum in scsi.h
#define MSG_NO_OPERATION 0x08

static struct {
    // Bitmap of all drives that have been imaged
    uint32_t drives_imaged;

    // Configuration from .ini
    uint8_t initiator_id;
    uint8_t max_retry_count;
    bool use_read10; // Always use read10 commands
    bool use_identify; // Select with ATN and send IDENTIFY before each command

    // SASI mode (InitiatorSASI=1): Xebec S1410 / WD1002-type controllers.
    // No messages, no INQUIRY / READ CAPACITY / START STOP UNIT, 4 byte sense,
    // geometry comes from the .ini file.
    bool sasi;
    bool sasi_send_params;          // Send Initialize Drive Characteristics (0x0C)
    bool sasi_select_initiator_bit; // Assert initiator ID bit during selection
    uint32_t sasi_cylinders;
    uint32_t sasi_heads;
    uint32_t sasi_sectors_per_track;
    uint32_t sasi_blocksize;
    uint32_t sasi_reduced_write;
    uint32_t sasi_precomp;
    uint32_t sasi_ecc_burst;
    uint8_t sasi_last_error;        // Error code (hex) from the last REQUEST SENSE
    bool sasi_params_ok;            // Controller has our drive characteristics (lost on bus reset)
    int sasi_recover_failures;      // Consecutive failed recovery attempts
    uint32_t sasi_alt_skipped;      // Sectors skipped on alternate / bad-flagged tracks
    bool sasi_skip_track_on_seek_error; // InitiatorSASISkipTrackOnSeekError: 0x15 skips the track
    uint32_t sasi_seek_skipped;     // Sectors skipped on tracks with seek error / ID not found

    // Is imaging a drive in progress, or are we scanning?
    bool imaging;

    // Information about currently selected drive
    int target_id;
    uint32_t sectorsize;
    uint32_t lcm_scsi_vs_sd_sectorsize;
    uint32_t sectorcount;
    uint32_t sectorcount_all;
    uint32_t sectors_done;
    uint32_t max_sector_per_transfer;
    uint32_t bad_sector_count;
    uint8_t ansi_version;
    uint8_t device_type;

    // Retry information for sector reads.
    // If a large read fails, retry is done sector-by-sector.
    int retrycount;
    uint32_t failposition;
    bool eject_when_done;
    bool removable;

    uint32_t removable_count[8];

    // VHD output format (opt-in via InitiatorVHD=1)
    bool use_vhd_format;

    // Negotiated bus width for targets
    int targetBusWidth[S2S_MAX_TARGETS];

    FsFile target_file;
} g_initiator_state;

extern SdFs SD;
static bool g_pause = false;

// The initiator never changes the source drive unless InitiatorAllowWrite = 1 is set.
// Without an SD card or .ini file this stays false.
static bool g_initiator_allow_write = false;

bool scsiInitiatorWriteAllowed()
{
    return g_initiator_allow_write;
}

// Position the image file at the given sector. SdFat cannot seek past the end of a
// file on FAT32, so skipped sectors at the end of the file are filled with zeros
// first; otherwise the following data would be written to the wrong offset.
static bool initiatorSeekSector(uint32_t sector)
{
    FsFile &file = g_initiator_state.target_file;
    uint64_t pos = (uint64_t)sector * g_initiator_state.sectorsize;
    uint64_t size = file.fileSize();
    if (pos > size)
    {
        static const uint8_t zeros[512] = {0};
        if (!file.seek(size))
        {
            logmsg("Image file seek to end failed at ", (int)(size / g_initiator_state.sectorsize));
            return false;
        }
        while (size < pos)
        {
            uint32_t n = (pos - size > sizeof(zeros)) ? sizeof(zeros) : (uint32_t)(pos - size);
            if (file.write(zeros, n) != n)
            {
                logmsg("Image file write failed while filling skipped sectors");
                return false;
            }
            size += n;
        }
    }

    if (!file.seek(pos))
    {
        logmsg("Image file seek failed to sector ", (int)sector);
        return false;
    }
    return true;
}

// Write zeros over skipped sectors [from, to) and leave the file positioned at 'to'.
// A failed multi-sector transfer may already have written stale buffer contents
// there, so seeking past them is not enough.
static bool initiatorZeroSectors(uint32_t from, uint32_t to)
{
    if (!initiatorSeekSector(from))
    {
        return false;
    }

    static const uint8_t zeros[512] = {0};
    uint64_t remaining = (uint64_t)(to - from) * g_initiator_state.sectorsize;
    while (remaining > 0)
    {
        uint32_t n = (remaining > sizeof(zeros)) ? sizeof(zeros) : (uint32_t)remaining;
        if (g_initiator_state.target_file.write(zeros, n) != n)
        {
            logmsg("Image file write failed while zeroing skipped sectors");
            return false;
        }
        remaining -= n;
    }
    return true;
}

// Stop imaging instead of writing data to a wrong position in the image file
static void initiatorStopImaging()
{
    logmsg("Imaging stopped at sector ", (int)g_initiator_state.sectors_done,
           " because the image file could not be positioned; the image is incomplete");
    g_initiator_state.imaging = false;
    g_initiator_state.target_file.close();
}

// Initialization of initiator mode
void scsiInitiatorInit()
{
    scsiHostPhyReset();

    g_initiator_state.initiator_id = ini_getl("SCSI", "InitiatorID", 7, CONFIGFILE);
    if (g_initiator_state.initiator_id > 7)
    {
        logmsg("InitiatorID set to illegal value in, ", CONFIGFILE, ", defaulting to 7");
        g_initiator_state.initiator_id = 7;
    }
    else
    {
        logmsg("InitiatorID set to ID ", g_initiator_state.initiator_id);
    }
    g_initiator_allow_write = ini_getbool("SCSI", "InitiatorAllowWrite", false, CONFIGFILE);
    if (g_initiator_allow_write)
    {
        logmsg("*************************************************************************");
        logmsg("  WARNING: InitiatorAllowWrite = 1, commands that WRITE to the source");
        logmsg("  drive are allowed (USB MSC writes, format commands)");
        logmsg("*************************************************************************");
    }
    else
    {
        logmsg("Initiator is read-only: write commands to the source drive are refused (InitiatorAllowWrite = 0)");
    }

    g_initiator_state.max_retry_count = ini_getl("SCSI", "InitiatorMaxRetry", 5, CONFIGFILE);
    g_initiator_state.use_read10 = ini_getbool("SCSI", "InitiatorUseRead10", false, CONFIGFILE);
    g_initiator_state.use_identify = ini_getbool("SCSI", "InitiatorIdentify", true, CONFIGFILE);
    g_initiator_state.use_vhd_format = ini_getbool("SCSI", "InitiatorVHD", false, CONFIGFILE);

    g_initiator_state.sasi = ini_getbool("SCSI", "InitiatorSASI", false, CONFIGFILE);
    if (g_initiator_state.sasi)
    {
        g_initiator_state.use_identify = false; // SASI has no message phases
        g_initiator_state.use_vhd_format = false;
        g_initiator_state.sasi_send_params = ini_getbool("SCSI", "InitiatorSASIInitParams", true, CONFIGFILE);
        g_initiator_state.sasi_select_initiator_bit = ini_getbool("SCSI", "InitiatorSASISelectInitiatorBit", false, CONFIGFILE);
        g_initiator_state.sasi_cylinders = ini_getl("SCSI", "InitiatorSASICylinders", 306, CONFIGFILE);
        g_initiator_state.sasi_heads = ini_getl("SCSI", "InitiatorSASIHeads", 2, CONFIGFILE);
        g_initiator_state.sasi_sectors_per_track = ini_getl("SCSI", "InitiatorSASISectorsPerTrack", 32, CONFIGFILE);
        g_initiator_state.sasi_blocksize = ini_getl("SCSI", "InitiatorSASIBlockSize", 256, CONFIGFILE);
        // Defaults are the Seagate ST-412 entry of the S1410 manual (appendix A), zero relative.
        // They only affect writing, which the initiator never does.
        g_initiator_state.sasi_reduced_write = ini_getl("SCSI", "InitiatorSASIReducedWriteCyl", 128, CONFIGFILE);
        g_initiator_state.sasi_precomp = ini_getl("SCSI", "InitiatorSASIPrecompCyl", 64, CONFIGFILE);
        g_initiator_state.sasi_ecc_burst = ini_getl("SCSI", "InitiatorSASIEccBurst", 11, CONFIGFILE);
        g_scsiHostDiag = true; // timeouts and bus state logging in byte writes
        g_scsiHostSasiTiming = true; // controller pauses within data phase, see scsi_accel_host_read()
        g_initiator_state.sasi_params_ok = false;
        g_initiator_state.sasi_recover_failures = 0;
        g_initiator_state.sasi_alt_skipped = 0;
        g_initiator_state.sasi_seek_skipped = 0;
        // Off by default: on a normal drive a single seek error must not skip a whole track
        g_initiator_state.sasi_skip_track_on_seek_error = ini_getbool("SCSI", "InitiatorSASISkipTrackOnSeekError", false, CONFIGFILE);
        if (g_initiator_state.sasi_skip_track_on_seek_error)
        {
            logmsg("SASI initiator: seek error / ID not found (0x15) skips the whole track (InitiatorSASISkipTrackOnSeekError = 1)");
        }
        g_scsiHostSwapCdMsg = ini_getbool("SCSI", "InitiatorSASISwapCdMsg", false, CONFIGFILE);
        if (g_scsiHostSwapCdMsg)
        {
            logmsg("SASI initiator: C/D and MSG inputs are interpreted swapped (InitiatorSASISwapCdMsg=1)");
        }
        logmsg("SASI initiator mode: ", (int)g_initiator_state.sasi_cylinders, " cyl x ",
               (int)g_initiator_state.sasi_heads, " heads x ",
               (int)g_initiator_state.sasi_sectors_per_track, " sectors x ",
               (int)g_initiator_state.sasi_blocksize, " bytes");
    }

    // treat initiator id as already imaged drive so it gets skipped
    g_initiator_state.drives_imaged = 1 << g_initiator_state.initiator_id;

    g_initiator_state.imaging = false;
    g_initiator_state.target_id = -1;
    g_initiator_state.sectorsize = 0;
    g_initiator_state.sectorcount = 0;
    g_initiator_state.sectors_done = 0;
    g_initiator_state.retrycount = 0;
    g_initiator_state.failposition = 0;
    g_initiator_state.max_sector_per_transfer = 512;
    g_initiator_state.ansi_version = 0;
    g_initiator_state.bad_sector_count = 0;
    g_initiator_state.device_type = SCSI_DEVICE_TYPE_DIRECT_ACCESS;
    g_initiator_state.removable = false;
    g_initiator_state.eject_when_done = false;
    memset(g_initiator_state.removable_count, 0, sizeof(g_initiator_state.removable_count));
    platform_led_breath(true, 0);

}

int scsiInitiatorGetOwnID()
{
    return g_initiator_state.initiator_id;
}

// Update progress bar LED during transfers
static void scsiInitiatorUpdateLed()
{
    // Update status indicator, the led blinks every 5 seconds and is on the longer the more data has been transferred
    const int period = 256;
    int phase = (millis() % period);
    int duty = (int64_t)g_initiator_state.sectors_done * period / g_initiator_state.sectorcount;

    // Minimum and maximum time to verify that the blink is visible
    if (duty < 50) duty = 50;
    if (duty > period - 50) duty = period - 50;

    if (phase <= duty)
    {
        LED_ON();
    }
    else
    {
        LED_OFF();
    }
}

uint8_t ejectButtonUpdate()
{
    // treat '1' to '0' transitions as reset actions
    static uint8_t previous = 0x00;
    uint8_t bitmask = platform_get_buttons() & EJECT_BTN_MASK;
    uint8_t ejectors = (previous ^ bitmask) & previous;
    previous = bitmask;
    return ejectors;
}

static void pollPauseOnEjectButton()
{
    if (ejectButtonUpdate() == 1)
    {
        logmsg("Eject button detected while in initiator mode, resetting state...");
        g_pause = true;
    }
}

void delay_with_poll(uint32_t ms)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms)
    {
        pollPauseOnEjectButton();
        platform_poll();
        delay(1);
    }
}

// Map a SCSI peripheral device type to a human-readable name.
// Returns nullptr for unsupported (non-block) types — the initiator skips
// those rather than attempting to clone them.
static const char *initiatorPeripheralTypeName(uint8_t device_type, bool removable)
{
    switch (device_type)
    {
        case SCSI_DEVICE_TYPE_DIRECT_ACCESS:  return removable ? "Removable " : "Disk";
        case SCSI_DEVICE_TYPE_SEQUENTIAL:     return "Sequential (Tape)";
        case SCSI_DEVICE_TYPE_WRITE_ONCE:     return "Write Once";
        case SCSI_DEVICE_TYPE_CD:             return "Optical (CD/DVD)";
        case SCSI_DEVICE_TYPE_MO:             return "Optical Memory (Magneto-optical)";
        case SCSI_DEVICE_TYPE_MEDIA_CHANGER:  return "Media Changer";
        case SCSI_DEVICE_TYPE_DISK_ARRAY:     return "Disk Array";
        default:                              return nullptr;
    }
}

// Check if VHD output should be used for the current target
static bool initiatorShouldWriteVhd()
{
    return g_initiator_state.use_vhd_format &&
           g_initiator_state.device_type == SCSI_DEVICE_TYPE_DIRECT_ACCESS &&
           !g_initiator_state.removable;
}

// High level logic of the initiator mode
void scsiInitiatorMainLoop()
{
    if (g_scsiHostPhyReset)
    {
        logmsg("Executing BUS RESET after aborted command");
        scsiHostPhyReset();
    }

#ifdef PLATFORM_MASS_STORAGE
    if (g_msc_initiator)
    {
        poll_msc_initiator();
        platform_run_msc();
        return;
    }
    else
    {
        // Upstream behaviour: USB MSC initiator mode without SD card or with InitiatorMSC = 1.
        // Without an SD card there is no .ini, so InitiatorAllowWrite stays 0 and MSC is read-only.
        if (!g_sdcard_present || ini_getbool("SCSI", "InitiatorMSC", false, CONFIGFILE))
        {
            // This delay allows the USB serial console to connect immediately to the host
            // It also decreases the delay in callback processing of MSC commands
            int32_t msc_init_delay = ini_getl("SCSI", "InitiatorMSCInitDelay", MSC_INIT_DELAY, CONFIGFILE);
            if (msc_init_delay != MSC_INIT_DELAY)
                logmsg("Initiator init delay set in ", CONFIGFILE ," to ", (int)msc_init_delay, " milliseconds");
            delay(msc_init_delay);

            // GT TODO
            logmsg("Entering USB MSC initiator mode");
            platform_enter_msc();
            setup_msc_initiator();
            return;
        }
    }
#endif

    if (!g_sdcard_present)
    {
        // Wait for SD card
        return;
    }

    pollPauseOnEjectButton();
    if (g_pause)
    {
        g_initiator_state.target_file.close();
        scsiInitiatorInit();
        logmsg("Initiator reset, pausing. Press eject to start initiator...");
        LED_OFF();
        platform_set_blink_status(false);
        platform_led_breath(true, PLATFORM_LED_PWM_BREATH_PERIOD_MS / 4);
        while(ejectButtonUpdate() != 1)
        {
            platform_poll();
            platform_reset_watchdog();
        }

        platform_led_breath(true, 0);
        g_pause = false;
        scsiHostPhyReset();
    }

    if (!g_initiator_state.imaging)
    {
        // Scan for SCSI drives one at a time
        g_initiator_state.target_id = (g_initiator_state.target_id + 1) % S2S_MAX_TARGETS;
        g_initiator_state.sectorsize = 0;
        g_initiator_state.sectorcount = 0;
        g_initiator_state.sectors_done = 0;
        g_initiator_state.retrycount = 0;
        g_initiator_state.failposition = 0;
        g_initiator_state.max_sector_per_transfer = 512;
        g_initiator_state.ansi_version = 0;
        g_initiator_state.bad_sector_count = 0;
        g_initiator_state.device_type = SCSI_DEVICE_TYPE_DIRECT_ACCESS;
        g_initiator_state.removable = false;
        g_initiator_state.eject_when_done = false;
        g_initiator_state.use_read10 = false;

        UIInitiatorScanning(g_initiator_state.target_id, g_initiator_state.initiator_id);
        
        if (!(g_initiator_state.drives_imaged & (1 << g_initiator_state.target_id)))
        {
            delay_with_poll(1000);

            uint8_t inquiry_data[36] = {0};

            LED_ON();

            // SASI controllers do not implement START STOP UNIT, use the
            // SASI specific probe sequence instead.
            bool startstopok = g_initiator_state.sasi ?
                scsiSasiProbe(g_initiator_state.target_id) :
                (scsiTestUnitReady(g_initiator_state.target_id) &&
                 scsiStartStopUnit(g_initiator_state.target_id, true));

#if defined(PLATFORM_MAX_BUS_WIDTH) && PLATFORM_MAX_BUS_WIDTH > 0
            if (startstopok && !g_initiator_state.sasi)
            {
                // Negotiate bus width
                // This is done before other commands just in case the target
                // happens to be in 16-bit mode. Only commands that have no
                // data phase can be used before this.
                int configBusWidth = ini_getl("SCSI", "InitiatorBusWidth", PLATFORM_MAX_BUS_WIDTH, CONFIGFILE);
                bool busWidthSet = scsiInitiatorSetBusWidth(g_initiator_state.target_id, configBusWidth);
                if (!busWidthSet && ini_haskey("SCSI", "InitiatorBusWidth", CONFIGFILE))
                {
                    logmsg("-- Failed to negotiate ", 8 << configBusWidth, " bit bus width that is forced in .ini file");
                    logmsg("-- Refusing to connect at lower bus width");
                    return;
                }
            }
#endif

            bool readcapok;
            bool inquiryok;
            if (g_initiator_state.sasi)
            {
                // SASI has no READ CAPACITY or INQUIRY, geometry is from .ini
                readcapok = startstopok;
                inquiryok = false;
                if (readcapok)
                {
                    g_initiator_state.sectorsize = g_initiator_state.sasi_blocksize;
                    g_initiator_state.sectorcount = g_initiator_state.sasi_cylinders
                        * g_initiator_state.sasi_heads * g_initiator_state.sasi_sectors_per_track;
                    g_initiator_state.use_read10 = false;
                    // Keep transfers well below the 256 block limit of READ6
                    g_initiator_state.max_sector_per_transfer = 64;
                    uint32_t max_by_buffer = sizeof(scsiDev.data) / g_initiator_state.sectorsize;
                    if (max_by_buffer < g_initiator_state.max_sector_per_transfer)
                    {
                        g_initiator_state.max_sector_per_transfer = max_by_buffer;
                    }
                    logmsg("SASI target ", g_initiator_state.target_id, " assumed geometry, vendor/product unknown");
                }
            }
            else
            {
                readcapok = startstopok &&
                    scsiInitiatorReadCapacity(g_initiator_state.target_id,
                                              &g_initiator_state.sectorcount,
                                              &g_initiator_state.sectorsize);

                inquiryok = startstopok &&
                    scsiInquiry(g_initiator_state.target_id, inquiry_data);
            }

            LED_OFF();

            uint64_t total_bytes = 0;
            if (readcapok)
            {
                logmsg("SCSI ID ", g_initiator_state.target_id,
                    " capacity ", (int)g_initiator_state.sectorcount,
                    " sectors x ", (int)g_initiator_state.sectorsize, " bytes");

                // Calculate the LCM of the SCSI medium's sector size vs SD sector size for transfer optimization
                uint32_t sectorsize = g_initiator_state.sectorsize;
                g_initiator_state.lcm_scsi_vs_sd_sectorsize = 0;
                if (sectorsize != SD_SECTOR_SIZE) 
                {
                    uint32_t gcd = std::gcd(sectorsize, SD_SECTOR_SIZE);
                    uint32_t factor = sectorsize / gcd;
                    uint32_t lcm_alignment = factor * SD_SECTOR_SIZE;
                    if (factor <= UINT32_MAX / SD_SECTOR_SIZE)
                        g_initiator_state.lcm_scsi_vs_sd_sectorsize = lcm_alignment;
                }

                UIInitiatorReadCapOk(g_initiator_state.target_id, (S2S_CFG_TYPE)g_initiator_state.device_type, g_initiator_state.sectorcount, g_initiator_state.sectorsize);
                
                g_initiator_state.sectorcount_all = g_initiator_state.sectorcount;

                total_bytes = (uint64_t)g_initiator_state.sectorcount * g_initiator_state.sectorsize;
                logmsg("Drive total size is ", (int)(total_bytes / (1024 * 1024)), " MiB");
                if (total_bytes >= 0xFFFFFFFF && SD.fatType() != FAT_TYPE_EXFAT)
                {
                    // GT TODO
                    // Note: the FAT32 limit is 4 GiB - 1 byte
                    logmsg("Target SCSI ID ", g_initiator_state.target_id, " image size is equal or larger than 4 GiB.");
                    logmsg("This is larger than the max filesize supported by SD card's filesystem");
                    logmsg("Please reformat the SD card with exFAT format to image this target");
                    g_initiator_state.drives_imaged |= 1 << g_initiator_state.target_id;
                    return;
                }
            }
            else if (startstopok)
            {
                // GT TODO
                logmsg("SCSI ID ", g_initiator_state.target_id, " responds but ReadCapacity command failed");
                logmsg("Possibly SCSI-1 drive? Attempting to read up to 1 GB.");
                g_initiator_state.sectorsize = 512;
                g_initiator_state.sectorcount = g_initiator_state.sectorcount_all = 2097152;
                g_initiator_state.max_sector_per_transfer = 128;
            }
            else
            {
                dbgmsg("Failed to connect to SCSI ID ", g_initiator_state.target_id);
                g_initiator_state.sectorsize = 0;
                g_initiator_state.sectorcount = g_initiator_state.sectorcount_all = 0;
            }

            char filename_base[12];
            strncpy(filename_base, "HD00_imaged", sizeof(filename_base));
            const char *filename_extension = ".hda";

            if (inquiryok)
            {
                char vendor[9], product[17], revision[5];
                g_initiator_state.device_type=inquiry_data[0] & 0x1f;
                g_initiator_state.ansi_version = inquiry_data[2] & 0x7;
                g_initiator_state.removable = !!(inquiry_data[1] & 0x80);
                g_initiator_state.eject_when_done = g_initiator_state.removable;
                memcpy(vendor, &inquiry_data[8], 8);
                vendor[8]=0;
                memcpy(product, &inquiry_data[16], 16);
                product[16]=0;
                memcpy(revision, &inquiry_data[32], 4);
                revision[4]=0;

                g_initiator_state.use_read10 = scsiInitiatorTestSupportsRead10(g_initiator_state.target_id, g_initiator_state.sectorsize);
                if(!g_initiator_state.use_read10)
                {
                    // READ6 command can transfer up to 256 sectors
                    g_initiator_state.max_sector_per_transfer = 256;
                }

                // Limit sectors per transfer based on buffer size
                uint32_t max_by_buffer = sizeof(scsiDev.data) / g_initiator_state.sectorsize;
                if (max_by_buffer < g_initiator_state.max_sector_per_transfer)
                {
                    g_initiator_state.max_sector_per_transfer = max_by_buffer;
                }


                logmsg("SCSI Version ", (int) g_initiator_state.ansi_version);
                logmsg("[SCSI", g_initiator_state.target_id,"]");
                logmsg("  Vendor = \"", vendor,"\"");
                logmsg("  Product = \"", product,"\"");
                logmsg("  Version = \"", revision,"\"");


                const char *typeName = initiatorPeripheralTypeName(
                    g_initiator_state.device_type, g_initiator_state.removable);

                if (typeName == nullptr)
                {
                    logmsg("  SCSI Peripheral device type id ", g_initiator_state.device_type, " unsupported. Skipping this device");
                    g_initiator_state.drives_imaged |= 1 << g_initiator_state.target_id;
                    return;
                }

                logmsg("  SCSI Device Type = ", typeName);

                if (g_initiator_state.device_type == SCSI_DEVICE_TYPE_CD)
                {
                    strncpy(filename_base, "CD00_imaged", sizeof(filename_base));
                    filename_extension = ".iso";
                }
                else if (g_initiator_state.device_type == SCSI_DEVICE_TYPE_MO)
                {
                    strncpy(filename_base, "MO00_imaged", sizeof(filename_base));
                    filename_extension = ".img";
                }
                else if (g_initiator_state.device_type != SCSI_DEVICE_TYPE_DIRECT_ACCESS)
                {
                    logmsg("  No specific handler for the device type, treating as Direct Access Device.");
                    g_initiator_state.device_type = SCSI_DEVICE_TYPE_DIRECT_ACCESS;
                }

                if (g_initiator_state.device_type == SCSI_DEVICE_TYPE_DIRECT_ACCESS && g_initiator_state.removable)
                {
                    strncpy(filename_base, "RM00_imaged", sizeof(filename_base));
                    filename_extension = ".img";
                }

                if (initiatorShouldWriteVhd())
                {
                    filename_extension = ".vhd";
                    logmsg("VHD output enabled for SCSI ID ", g_initiator_state.target_id);
                }
            }

            if (g_initiator_state.eject_when_done && g_initiator_state.removable_count[g_initiator_state.target_id] == 0)
            {
                g_initiator_state.removable_count[g_initiator_state.target_id] = 1;
            }

            if (g_initiator_state.sectorcount > 0)
            {
                char filename[32] = {0};
                filename_base[2] = scsiEncodeID(g_initiator_state.target_id);
                if (g_initiator_state.eject_when_done)
                {
                    auto removable_count = g_initiator_state.removable_count[g_initiator_state.target_id];
                    snprintf(filename, sizeof(filename), "%s(%lu)%s",filename_base, removable_count, filename_extension);
                }
                else
                {
                    snprintf(filename, sizeof(filename), "%s%s", filename_base, filename_extension);
                }
                static int handling = -1;
                if (handling == -1)
                {
                    handling = ini_getl("SCSI", "InitiatorImageHandling", 0, CONFIGFILE);
                }
                // Stop if a file already exists
                if (handling == 0)
                {
                    if (SD.exists(filename))
                    {
                        // GT TODO
                        logmsg("File, ", filename, ", already exists, InitiatorImageHandling set to stop if file exists.");
                        g_initiator_state.drives_imaged |= (1 << g_initiator_state.target_id);
                        return;
                    }
                }
                // Create a new copy to the file 002-999
                else if (handling == 1)
                {
                    for (uint32_t i = 1; i <= 1000; i++)
                    {
                        if (i == 1)
                        {
                            if (SD.exists(filename))
                                continue;
                            break;
                        }
                        else if(i >= 1000)
                        {
                            // GT TODO
                            logmsg("Max images created from SCSI ID ", g_initiator_state.target_id, ", skipping image creation");
                            g_initiator_state.drives_imaged |= (1 << g_initiator_state.target_id);
                            return;
                        }
                        char filename_copy[6] = {0};
                        if (g_initiator_state.eject_when_done)
                        {
                            auto removable_count = g_initiator_state.removable_count[g_initiator_state.target_id];
                            snprintf(filename, sizeof(filename), "%s(%lu)-%03lu%s", filename_base, removable_count, i, filename_extension);
                        }
                        else
                        {
                            snprintf(filename, sizeof(filename), "%s-%03lu%s", filename_base, i, filename_extension);
                        }
                        snprintf(filename_copy, sizeof(filename_copy), "-%03lu", i);
                        if (SD.exists(filename))
                            continue;
                        break;
                    }

                }
                // overwrite file if it exists
                else if (handling == 2)
                {
                    if (SD.exists(filename))
                    {
                        // GT TODO
                        logmsg("File, ",filename, " already exists, InitiatorImageHandling set to overwrite file");
                        SD.remove(filename);
                    }
                }
                // InitiatorImageHandling invalid setting
                else
                {
                    static bool invalid_logged_once = false;
                    if (!invalid_logged_once)
                    {
                        logmsg("InitiatorImageHandling is set to, ", handling, ", which is invalid");
                        invalid_logged_once = true;
                    }
                    return;
                }

                uint64_t vhd_overhead = initiatorShouldWriteVhd() ? VHD_FOOTER_SIZE : 0;
                uint64_t sd_card_free_bytes = (uint64_t)SD.vol()->freeClusterCount() * SD.vol()->bytesPerCluster();
                if (sd_card_free_bytes < total_bytes + vhd_overhead)
                {
                    // GT TODO
                    logmsg("SD Card only has ", (int)(sd_card_free_bytes / (1024 * 1024)),
                           " MiB - not enough free space to image SCSI ID ", g_initiator_state.target_id);
                    g_initiator_state.drives_imaged |= 1 << g_initiator_state.target_id;
                    return;
                }

                g_initiator_state.target_file = SD.open(filename, O_WRONLY | O_CREAT | O_TRUNC);
                if (!g_initiator_state.target_file.isOpen())
                {
                    logmsg("Failed to open file for writing: ", filename);
                    return;
                }

                if (SD.fatType() == FAT_TYPE_EXFAT)
                {
                    // Only preallocate on exFAT, on FAT32 preallocating can result in false garbage data in the
                    // file if write is interrupted.
                    logmsg("Preallocating image file");
                    g_initiator_state.target_file.preAllocate(
                        (uint64_t)g_initiator_state.sectorcount * g_initiator_state.sectorsize + vhd_overhead
                    );
                }

                UIInitiatorTargetFilename(g_initiator_state.target_id, filename);

                logmsg("Starting to copy drive data to ", filename);
                g_initiator_state.imaging = true;
            }
        }
    }
    else
    {
        // Copy sectors from SCSI drive to file
        if (g_initiator_state.sectors_done >= g_initiator_state.sectorcount)
        {
            if (!g_initiator_state.sasi)
            {
                scsiStartStopUnit(g_initiator_state.target_id, false);
            }
            // Sectors skipped at the very end are not written yet: pad the image to full size
            if (!initiatorSeekSector(g_initiator_state.sectorcount))
            {
                logmsg("WARNING: could not pad the image to its full size");
            }
            g_initiator_state.target_file.flush();

            logmsg("Finished imaging drive with id ", g_initiator_state.target_id);
            LED_OFF();

            UIInitiatorImagingComplete(g_initiator_state.target_id);
            
            if (g_initiator_state.sectorcount != g_initiator_state.sectorcount_all)
            {

                // GT TODO
                logmsg("NOTE: Image size was limited to first 4 GiB due to SD card filesystem limit");
                logmsg("Please reformat the SD card with exFAT format to image this drive fully");
            }

            if(g_initiator_state.bad_sector_count != 0)
            {
                // GT TODO
                logmsg("NOTE: There were ",  (int) g_initiator_state.bad_sector_count, " bad sectors that could not be read off this drive.");
            }

            if (g_initiator_state.sasi && g_initiator_state.sasi_alt_skipped != 0)
            {
                logmsg("NOTE: ", (int)g_initiator_state.sasi_alt_skipped,
                       " sectors on alternate or bad-flagged tracks were skipped (left as zeros in the image).");
            }

            if (g_initiator_state.sasi && g_initiator_state.sasi_seek_skipped != 0)
            {
                logmsg("NOTE: ", (int)g_initiator_state.sasi_seek_skipped,
                       " sectors on tracks with seek error / ID not found were skipped (left as zeros in the image).");
            }

            if (!g_initiator_state.eject_when_done)
            {
                // GT TODO
                logmsg("Marking SCSI ID, ", g_initiator_state.target_id, ", as imaged, wont ask it again.");
                g_initiator_state.drives_imaged |= (1 << g_initiator_state.target_id);
            }

            // Write VHD footer if enabled for this target
            if (initiatorShouldWriteVhd())
            {
                uint64_t raw_bytes = (uint64_t)g_initiator_state.sectorcount * g_initiator_state.sectorsize;
                uint8_t vhd_footer[VHD_FOOTER_SIZE];
                // Use 0 for timestamp — embedded device has no RTC epoch reference
                vhd_build_fixed_footer(vhd_footer, raw_bytes,
                                       g_initiator_state.sectorcount, 0,
                                       g_initiator_state.target_id);
                if (g_initiator_state.target_file.write(vhd_footer, VHD_FOOTER_SIZE) == VHD_FOOTER_SIZE)
                {
                    logmsg("VHD footer written successfully");
                }
                else
                {
                    logmsg("WARNING: Failed to write VHD footer");
                }
            }


            g_initiator_state.imaging = false;
            g_initiator_state.target_file.close();
            return;
        }

        scsiInitiatorUpdateLed();

        // How many sectors to read in one batch?
        int numtoread = g_initiator_state.sectorcount - g_initiator_state.sectors_done;
        if (numtoread > g_initiator_state.max_sector_per_transfer)
            numtoread = g_initiator_state.max_sector_per_transfer;

        // Retry sector-by-sector after failure
        if (g_initiator_state.sectors_done < g_initiator_state.failposition)
            numtoread = 1;

        if (g_initiator_state.sasi)
        {
            // A bus reset makes the controller forget the drive characteristics (0x0C) and
            // fall back to its default geometry, which silently reads wrong sectors.
            // Never read before the parameters have been sent again successfully.
            if (!g_initiator_state.sasi_params_ok)
            {
                if (!scsiSasiRecover(g_initiator_state.target_id))
                {
                    g_initiator_state.sasi_recover_failures++;
                    if (g_initiator_state.sasi_recover_failures >= 10)
                    {
                        logmsg("SASI: controller did not accept drive characteristics after ",
                               g_initiator_state.sasi_recover_failures, " attempts, imaging stopped at sector ",
                               (int)g_initiator_state.sectors_done, " to avoid reading with wrong geometry");
                        g_initiator_state.imaging = false;
                        g_initiator_state.target_file.close();
                        return;
                    }
                    delay_with_poll(1000);
                    return;
                }
                g_initiator_state.sasi_recover_failures = 0;
            }
            g_initiator_state.sasi_last_error = 0;
        }

        uint32_t time_start = millis();
        bool status = scsiInitiatorReadDataToFile(g_initiator_state.target_id,
            g_initiator_state.sectors_done, numtoread, g_initiator_state.sectorsize,
            g_initiator_state.target_file);

        if (!status)
        {
            logmsg("Failed to transfer ", numtoread, " sectors starting at ", (int)g_initiator_state.sectors_done);

            UIInitiatorFailedToTransfer(g_initiator_state.target_id);

            if (g_initiator_state.sasi)
            {
                uint8_t e = g_initiator_state.sasi_last_error;
                bool seek_skip = (e == 0x15 && g_initiator_state.sasi_skip_track_on_seek_error);
                if (seek_skip)
                {
                    // WD1010-based controllers report "ID not found" (e.g. an unformatted
                    // track) as seek error 0x15. Recalibrate before the next read.
                    g_initiator_state.sasi_params_ok = false;
                }
                if (e == 0x1C || e == 0x19 || seek_skip)
                {
                    // 0x1C: direct access to an alternate track is refused by the controller
                    // (S1410A), its data is read through the bad track it replaces.
                    // 0x19: track flagged bad. Retrying cannot help, skip the track.
                    g_initiator_state.retrycount = 0;
                    if (numtoread > 1)
                    {
                        logmsg("SASI: alternate or bad track inside batch, continuing sector-by-sector");
                        g_initiator_state.failposition = g_initiator_state.sectors_done + numtoread;
                    }
                    else
                    {
                        uint32_t spt = g_initiator_state.sasi_sectors_per_track;
                        uint32_t next = (g_initiator_state.sectors_done / spt + 1) * spt;
                        if (next > g_initiator_state.sectorcount) next = g_initiator_state.sectorcount;
                        uint32_t track = g_initiator_state.sectors_done / spt;
                        logmsg("SASI: ", (e == 0x1C) ? "alternate track (direct access refused)" :
                                         (e == 0x19) ? "bad track flag" : "seek error / ID not found (0x15)",
                               ", skipping sectors ", (int)g_initiator_state.sectors_done, "-", (int)(next - 1),
                               " (cylinder ", (int)(track / g_initiator_state.sasi_heads),
                               " head ", (int)(track % g_initiator_state.sasi_heads), ")");
                        if (seek_skip)
                            g_initiator_state.sasi_seek_skipped += next - g_initiator_state.sectors_done;
                        else
                            g_initiator_state.sasi_alt_skipped += next - g_initiator_state.sectors_done;
                        if (!initiatorZeroSectors(g_initiator_state.sectors_done, next)) { initiatorStopImaging(); return; }
                        g_initiator_state.sectors_done = next;
                    }
                    if (!initiatorSeekSector(g_initiator_state.sectors_done)) { initiatorStopImaging(); return; }
                    return;
                }

                // Any other failure may have involved a bus reset: re-send parameters before next read
                g_initiator_state.sasi_params_ok = false;
            }

            // SASI: when a single sector fails with a media error (ID field read error,
            // uncorrectable data, address mark / sector not found, bad track) retrying
            // rarely helps: allow just one more attempt before skipping it.
            uint32_t max_retry = g_initiator_state.max_retry_count;
            if (g_initiator_state.sasi && numtoread == 1)
            {
                uint8_t e = g_initiator_state.sasi_last_error;
                if ((e >= 0x10 && e <= 0x14) || e == 0x19)
                {
                    if (max_retry > 1) max_retry = 1;
                }
            }

            if (g_initiator_state.retrycount < max_retry)
            {
                logmsg("Retrying.. ", g_initiator_state.retrycount + 1, "/", (int) max_retry);
                delay_with_poll(200);
                // This reset causes some drives to hang and seems to have no effect if left off.
                // scsiHostPhyReset();
                delay_with_poll(200);

                g_initiator_state.retrycount++;
                if (!initiatorSeekSector(g_initiator_state.sectors_done)) { initiatorStopImaging(); return; }

                if (g_initiator_state.retrycount > 1 && numtoread > 1)
                {
                    // GT TODO
                    logmsg("Multiple failures, retrying sector-by-sector");
                    g_initiator_state.failposition = g_initiator_state.sectors_done + numtoread;
                }

                UIInitiatorRetry(g_initiator_state.target_id);
            }
            else
            {
                logmsg("Retry limit exceeded, skipping one sector");
                g_initiator_state.retrycount = 0;
                // Zero the skipped sector: a failed transfer may have left stale data there
                if (!initiatorZeroSectors(g_initiator_state.sectors_done, g_initiator_state.sectors_done + 1)) { initiatorStopImaging(); return; }
                g_initiator_state.sectors_done++;
                g_initiator_state.bad_sector_count++;

                UIInitiatorSkippedSector(g_initiator_state.target_id);
            }
        }
        else
        {
            g_initiator_state.retrycount = 0;
            g_initiator_state.sectors_done += numtoread;
            g_initiator_state.target_file.flush();

            int speed_kbps = numtoread * g_initiator_state.sectorsize / (millis() - time_start);
            logmsg("SCSI read succeeded, sectors done: ",
                  (int)g_initiator_state.sectors_done, " / ", (int)g_initiator_state.sectorcount,
                  " speed ", speed_kbps, " kB/s - ", 
                  (int)(100 * (int64_t)g_initiator_state.sectors_done / g_initiator_state.sectorcount), "%");

            UIInitiatorProgress(g_initiator_state.target_id, millis() - time_start, g_initiator_state.sectors_done, numtoread);
        }
    }
}

/*************************************
 * Low level command implementations *
 *************************************/

// Debug log of the raw bus line states (BSY REQ C/D I/O MSG and data bus, 1 = asserted)
static void logRawBusState(const char *tag)
{
    uint32_t data = 0;
    uint32_t s = scsiHostPhyGetRawState(&data);
    char db[16];
    for (int i = 0; i < 9; i++)
    {
        db[i] = (data & (1 << (8 - i))) ? '1' : '0';
    }
    db[9] = 0;
    dbgmsg("------ ", tag, ": BSY=", (int)(s & 1), " REQ=", (int)((s >> 1) & 1),
           " CD=", (int)((s >> 2) & 1), " IO=", (int)((s >> 3) & 1),
           " MSG=", (int)((s >> 4) & 1), " DB8..DB0=", db);
}

int scsiInitiatorRunCommand(int target_id,
                            const uint8_t *command, size_t cmdLen,
                            uint8_t *bufIn, size_t bufInLen,
                            const uint8_t *bufOut, size_t bufOutLen,
                            bool returnDataPhase, uint32_t timeout)
{
    // Unless InitiatorAllowWrite = 1, the initiator must never change the source drive.
    // Only commands that do not write to the medium are allowed; everything else
    // (WRITE, FORMAT, FORMAT TRACK/BAD TRACK/ALTERNATE, WRITE BUFFER, MODE SELECT ...)
    // is refused here, which also covers the USB MSC initiator mode.
    // 0x0C (SASI Initialize Drive Characteristics) only sets controller RAM parameters.
    if (!g_initiator_allow_write)
    {
        switch (command[0])
        {
            case 0x00: // TEST UNIT READY / TEST DRIVE READY
            case 0x01: // REZERO UNIT / RECALIBRATE
            case 0x03: // REQUEST SENSE
            case 0x08: // READ(6)
            case 0x0C: // SASI: INITIALIZE DRIVE CHARACTERISTICS
            case 0x12: // INQUIRY
            case 0x1A: // MODE SENSE(6)
            case 0x1B: // START STOP UNIT
            case 0x25: // READ CAPACITY
            case 0x28: // READ(10)
                break;
            default:
                logmsg("READ-ONLY: refusing command 0x", bytearray(command, 1), " to target ", target_id,
                       " (set InitiatorAllowWrite = 1 to allow writes)");
                return -1;
        }
    }

    // SCSI-3 targets take the LUN from the IDENTIFY message and reject
    // anything selected without one (ILLEGAL REQUEST / LUN NOT SUPPORTED).
    // The target only offers MESSAGE_OUT if we select with ATN asserted.
    bool send_identify = g_initiator_state.use_identify;
    bool identify_sent = false;

    if (send_identify)
    {
        scsiHostPhySetATN(true);
    }

    // SASI hosts typically assert only the target ID bit during selection
    uint8_t select_initiator_id = (g_initiator_state.sasi && !g_initiator_state.sasi_select_initiator_bit) ?
        SCSI_HOST_SELECT_NO_INITIATOR_ID : g_initiator_state.initiator_id;

    if (!scsiHostPhySelect(target_id, select_initiator_id))
    {
        scsiHostPhySetATN(false);
        dbgmsg("------ Target ", target_id, " did not respond");
        scsiHostPhyRelease();
        return -1;
    }

    SCSI_PHASE phase;
    int status = -1;
    uint32_t start = millis();

    if (g_initiator_state.sasi)
    {
        // Diagnostics: how does the bus look right after the target answered selection?
        logRawBusState("SASI bus after selection");
        delayMicroseconds(100);
        logRawBusState("SASI bus after selection +100us");
    }
    uint32_t unknown_phase_since = 0;
    int last_unknown_phase = -100;

    while ((phase = (SCSI_PHASE)scsiHostPhyGetPhase()) != BUS_FREE)
    {
        // If explicit timeout is specified, prevent watchdog from triggering too early.
        if ((uint32_t)(millis() - start) < timeout)
        {
            platform_reset_watchdog();
        }

        platform_poll();

        // A phase that is not a legal SCSI phase (e.g. MSG alone) is never handled below.
        // Log it with the raw line states and give up instead of spinning until the watchdog fires.
        if (phase != BUS_BUSY && phase != MESSAGE_IN && phase != MESSAGE_OUT && phase != COMMAND &&
            phase != DATA_IN && phase != DATA_OUT && phase != STATUS)
        {
            if (phase != last_unknown_phase)
            {
                last_unknown_phase = phase;
                unknown_phase_since = millis();
                logmsg("------ Illegal bus phase ", (int)phase, " from target ", target_id);
                logRawBusState("Illegal phase, bus");
            }
            else if ((uint32_t)(millis() - unknown_phase_since) > 2000)
            {
                logmsg("------ Bus stuck in illegal phase ", (int)phase, " for 2 s, giving up");
                logRawBusState("Stuck bus");
                status = -4;
                break;
            }
            continue;
        }
        last_unknown_phase = -100;

        if (phase == MESSAGE_IN)
        {
            uint8_t msg = 0;
            scsiHostRead(&msg, 1);

            if (msg == MSG_COMMAND_COMPLETE)
            {
                break;
            }
        }
        else if (phase == MESSAGE_OUT)
        {
            // Negate ATN before the byte is acknowledged, otherwise the target
            // keeps asking for more messages.
            scsiHostPhySetATN(false);

            // IDENTIFY, LUN 0, no disconnect privilege. A target that asks
            // again gets NO OPERATION rather than a second IDENTIFY.
            uint8_t msg = identify_sent ? MSG_NO_OPERATION : 0x80;
            identify_sent = true;
            scsiHostWrite(&msg, 1);
        }
        else if (phase == COMMAND)
        {
            if (send_identify && !identify_sent)
            {
                // Target went straight to COMMAND, it does not use messages
                scsiHostPhySetATN(false);
            }
            scsiHostWrite(command, cmdLen);
        }
        else if (phase == DATA_IN)
        {
            if (returnDataPhase) return 0;
            if (bufInLen == 0)
            {
                logmsg("DATA_IN phase but no data to receive!");
                status = -3;
                break;
            }

            scsiHostSetBusWidth(g_initiator_state.targetBusWidth[target_id]);
            uint32_t readCount = scsiHostRead(bufIn, bufInLen);
            scsiHostSetBusWidth(0);
            if (readCount != bufInLen)
            {
                logmsg("scsiHostRead failed, tried to read ", (int)bufInLen, " bytes, got ", (int)readCount);
                status = -2;
                break;
            }
        }
        else if (phase == DATA_OUT)
        {
            if (returnDataPhase) return 0;
            if (bufOutLen == 0)
            {
                logmsg("DATA_OUT phase but no data to send!");
                status = -3;
                break;
            }

            scsiHostSetBusWidth(g_initiator_state.targetBusWidth[target_id]);
            uint32_t writeCount = scsiHostWrite(bufOut, bufOutLen);
            scsiHostSetBusWidth(0);
            if (writeCount != bufOutLen)
            {
                logmsg("scsiHostWrite failed, was writing ", bytearray(bufOut, bufOutLen), " return value ", (int)writeCount);
                status = -2;
                break;
            }
        }
        else if (phase == STATUS)
        {
            uint8_t tmp = -1;
            scsiHostRead(&tmp, 1);
            status = tmp;
            dbgmsg("------ STATUS: ", tmp);
        }
    }

    scsiHostPhySetATN(false);
    scsiHostWaitBusFree();

    return status;
}

int scsiInitiatorMessage(int target_id,
    const uint8_t *msgOut, size_t msgOutLen,
    uint8_t *msgIn, size_t msgInBufSize, size_t *msgInLen,
    uint32_t timeout)
{
    uint8_t command[6] = {0x00, 0, 0, 0, 0, 0};

    scsiHostPhySetATN(true);

    if (!scsiHostPhySelect(target_id, g_initiator_state.initiator_id))
    {
        dbgmsg("------ Target ", target_id, " did not respond");
        scsiHostPhyRelease();
        return -1;
    }

    size_t dummy;
    if (!msgInLen) msgInLen = &dummy;
    *msgInLen = 0;

    size_t msgOutSent = 0;

    SCSI_PHASE phase;
    int status = -1;
    uint32_t start = millis();
    while ((phase = (SCSI_PHASE)scsiHostPhyGetPhase()) != BUS_FREE)
    {
        // If explicit timeout is specified, prevent watchdog from triggering too early.
        if ((uint32_t)(millis() - start) < timeout)
        {
            platform_reset_watchdog();
        }

        platform_poll();

        if (phase == MESSAGE_IN)
        {
            uint8_t msg = 0;
            scsiHostRead(&msg, 1);

            if (*msgInLen < msgInBufSize)
            {
                msgIn[*msgInLen] = msg;
                *msgInLen += 1;
            }

            if (status != -1 && msg == MSG_COMMAND_COMPLETE)
            {
                break;
            }
        }
        else if (phase == MESSAGE_OUT)
        {
            if (msgOutSent < msgOutLen)
            {
                scsiHostWrite(&msgOut[msgOutSent++], 1);
                if (msgOutSent >= msgOutLen)
                {
                    // End of MESSAGE_OUT phase
                    // Note that target may switch to MESSAGE_IN earlier than this
                    scsiHostPhySetATN(false);
                }
            }
        }
        else if (phase == COMMAND)
        {
            scsiHostWrite(command, sizeof(command));
        }
        else if (phase == STATUS)
        {
            uint8_t tmp = -1;
            scsiHostRead(&tmp, 1);
            status = tmp;
            dbgmsg("------ STATUS: ", tmp);
        }
    }

    scsiHostWaitBusFree();

    return status;
}

bool scsiInitiatorTestSupportsRead10(int target_id, uint32_t sectorsize)
{
    if (ini_haskey("SCSI", "InitiatorUseRead10", CONFIGFILE))
    {
        return ini_getbool("SCSI", "InitiatorUseRead10", false, CONFIGFILE);
    }

    uint8_t command[10] = {0x28, 0x00, 0, 0, 0, 0, 0, 0, 1, 0}; // READ10, LBA 0, 1 sector
    int status = scsiInitiatorRunCommand(target_id, command, sizeof(command),
        scsiDev.data, sectorsize, NULL, 0);

    if (status == 0)
    {
        dbgmsg("Target supports READ10 command");
        return true;
    }
    else
    {
        dbgmsg("Target does not support READ10 command");
        return false;
    }
}

bool scsiInitiatorReadCapacity(int target_id, uint32_t *sectorcount, uint32_t *sectorsize)
{
    uint8_t command[10] = {0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t response[8] = {0};
    int status = scsiInitiatorRunCommand(target_id,
                                         command, sizeof(command),
                                         response, sizeof(response),
                                         NULL, 0);

    if (status == 0)
    {
        *sectorcount = ((uint32_t)response[0] << 24)
                    | ((uint32_t)response[1] << 16)
                    | ((uint32_t)response[2] <<  8)
                    | ((uint32_t)response[3] <<  0);

        *sectorcount += 1; // SCSI reports last sector address

        *sectorsize = ((uint32_t)response[4] << 24)
                    | ((uint32_t)response[5] << 16)
                    | ((uint32_t)response[6] <<  8)
                    | ((uint32_t)response[7] <<  0);

        return true;
    }
    else if (status == 2)
    {
        uint8_t sense_key;
        scsiRequestSense(target_id, &sense_key);
        scsiLogInitiatorCommandFailure("READ CAPACITY", target_id, status, sense_key);
        return false;
    }
    else
    {
        *sectorcount = *sectorsize = 0;
        return false;
    }
}

// Execute REQUEST SENSE command to get more information about error status
bool scsiRequestSense(int target_id, uint8_t *sense_key, uint8_t *sense_asc, uint8_t *sense_ascq)
{
    if (g_initiator_state.sasi)
    {
        // SASI (Xebec S1410) returns exactly 4 bytes and has no allocation length:
        //   byte 0: bit 7 address valid, bits 5-4 error type, bits 3-0 error code
        //   bytes 1-3: drive select bit + logical block address
        uint8_t sasi_command[6] = {0x03, 0, 0, 0, 0, 0};
        uint8_t sasi_response[4] = {0};
        int sasi_status = scsiInitiatorRunCommand(target_id,
                                                  sasi_command, sizeof(sasi_command),
                                                  sasi_response, sizeof(sasi_response),
                                                  NULL, 0);
        uint8_t error = sasi_response[0] & 0x7F;
        dbgmsg("SASI RequestSense response: ", bytearray(sasi_response, 4),
               " error type ", (int)((error >> 4) & 3), " code ", (int)(error & 0xF));

        g_initiator_state.sasi_last_error = error;

        // Map to the closest SCSI sense key for the callers.
        // 0x18 = correctable data error: the controller has already corrected the data.
        if (sense_key) *sense_key = (error == 0) ? NO_SENSE : (error == 0x18) ? RECOVERED_ERROR : MEDIUM_ERROR;
        if (sense_asc) *sense_asc = error;
        if (sense_ascq) *sense_ascq = 0;
        return sasi_status == 0;
    }

    uint8_t command[6] = {0x03, 0, 0, 0, 18, 0};
    uint8_t response[18] = {0};

    int status = scsiInitiatorRunCommand(target_id,
                                         command, sizeof(command),
                                         response, sizeof(response),
                                         NULL, 0);

    dbgmsg("RequestSense response: ", bytearray(response, 18),
        " sense_key ", (int)(response[2] & 0xF),
        " asc ", response[12], " ascq ", response[13]);

    if (sense_key) *sense_key = response[2] & 0xF;
    if (sense_asc) *sense_asc = response[12];
    if (sense_ascq) *sense_ascq = response[13];

    return status == 0;
}

// SASI error recovery after a failed transfer: the same bring-up as at start
// (TEST DRIVE READY until ready, drive characteristics 0x0C, RECALIBRATE).
// Reading is allowed again only when all of them succeed.
bool scsiSasiRecover(int target_id)
{
    g_initiator_state.sasi_params_ok = false;
    if (!scsiSasiProbe(target_id))
    {
        logmsg("SASI recovery: drive bring-up failed, not reading yet");
        return false;
    }

    logmsg("SASI recovery: drive characteristics sent and drive recalibrated");
    return true;
}

// SASI (Xebec S1410, WD1002) drive bring-up:
// TEST DRIVE READY, optionally INITIALIZE DRIVE CHARACTERISTICS (0x0C) with
// the geometry from the .ini file, then RECALIBRATE.
bool scsiSasiProbe(int target_id)
{
    // S1410 manual 4.7: issue Test Drive Ready repeatedly (with a time-out)
    // until the drive reports ready, e.g. after spin-up.
    uint8_t tur[6] = {0x00, 0, 0, 0, 0, 0};
    int status = -1;
    for (int tries = 0; tries < 30; tries++)
    {
        status = scsiInitiatorRunCommand(target_id, tur, sizeof(tur), NULL, 0, NULL, 0);
        if (status == -1)
        {
            // No response to selection
            return false;
        }
        else if (status == 0)
        {
            break;
        }

        uint8_t sense_key;
        uint8_t error;
        scsiRequestSense(target_id, &sense_key, &error);
        dbgmsg("SASI TEST DRIVE READY failed, status ", status, " error ", (int)error);
        delay_with_poll(1000);
    }

    if (status != 0)
    {
        logmsg("SASI drive on target ", target_id, " did not become ready");
        return false;
    }

    if (g_initiator_state.sasi_send_params)
    {
        uint8_t cmd[6] = {0x0C, 0, 0, 0, 0, 0};
        uint8_t params[8] = {
            (uint8_t)(g_initiator_state.sasi_cylinders >> 8), (uint8_t)g_initiator_state.sasi_cylinders,
            (uint8_t)g_initiator_state.sasi_heads,
            (uint8_t)(g_initiator_state.sasi_reduced_write >> 8), (uint8_t)g_initiator_state.sasi_reduced_write,
            (uint8_t)(g_initiator_state.sasi_precomp >> 8), (uint8_t)g_initiator_state.sasi_precomp,
            (uint8_t)g_initiator_state.sasi_ecc_burst
        };
        status = scsiInitiatorRunCommand(target_id, cmd, sizeof(cmd), NULL, 0, params, sizeof(params));
        if (status != 0)
        {
            uint8_t sense_key;
            uint8_t error;
            scsiRequestSense(target_id, &sense_key, &error);
            logmsg("SASI Initialize Drive Characteristics failed, status ", status, " error ", (int)error);
            return false;
        }
    }

    uint8_t recal[6] = {0x01, 0, 0, 0, 0, 0};
    status = scsiInitiatorRunCommand(target_id, recal, sizeof(recal), NULL, 0, NULL, 0, false, 60000);
    if (status != 0)
    {
        uint8_t sense_key;
        uint8_t error;
        scsiRequestSense(target_id, &sense_key, &error);
        logmsg("SASI Recalibrate failed, status ", status, " error ", (int)error);
        return false;
    }

    g_initiator_state.sasi_params_ok = true;
    return true;
}

// Execute UNIT START STOP command to load/unload media
bool scsiStartStopUnit(int target_id, bool start)
{
    uint8_t command[6] = {0x1B, 0x1, 0, 0, 0, 0};
    uint8_t response[4] = {0};

    if (start)
    {
        command[4] |= 1; // Start
        command[1] = 0;  // Immediate
    }
    else // stop
    {
        if(g_initiator_state.eject_when_done)
        {
            logmsg("Ejecting media on SCSI ID: ", target_id);
            g_initiator_state.removable_count[g_initiator_state.target_id]++;
            command[4] = 0b00000010; // eject(6), stop(7).
        }
    }

    uint32_t timeout = 60000; // Some drives can take long to initialize
    int status = scsiInitiatorRunCommand(target_id,
                                         command, sizeof(command),
                                         response, sizeof(response),
                                         NULL, 0, false, timeout);

    if (status == 2)
    {
        uint8_t sense_key;
        scsiRequestSense(target_id, &sense_key);
        scsiLogInitiatorCommandFailure("START STOP UNIT", target_id, status, sense_key);

        if (sense_key == NOT_READY)
        {
            dbgmsg("--- Device reports NOT_READY, running STOP to attempt restart");
            // Some devices will only leave NOT_READY state after they have been
            // commanded to stop state first.
            delay(1000);
            uint8_t cmd_stop[6] = {0x1B, 0x1, 0, 0, 0, 0};
            scsiInitiatorRunCommand(target_id,
                                    cmd_stop, sizeof(cmd_stop),
                                    response, sizeof(response),
                                    NULL, 0);
        }
    }

    return status == 0;
}

// Execute INQUIRY command
bool scsiInquiry(int target_id, uint8_t inquiry_data[36])
{
    uint8_t command[6] = {0x12, 0, 0, 0, 36, 0};
    int status = scsiInitiatorRunCommand(target_id,
                                         command, sizeof(command),
                                         inquiry_data, 36,
                                         NULL, 0);
    return status == 0;
}

// Execute TEST UNIT READY command and handle unit attention state
bool scsiTestUnitReady(int target_id)
{
    for (int retries = 0; retries < 2; retries++)
    {
        uint8_t command[6] = {0x00, 0, 0, 0, 0, 0};
        int status = scsiInitiatorRunCommand(target_id,
                                            command, sizeof(command),
                                            NULL, 0,
                                            NULL, 0);

        if (status == 0)
        {
            return true;
        }
        else if (status == -1)
        {
            // No response to select
            return false;
        }
        else if (status == 2)
        {
            uint8_t sense_key;
            scsiRequestSense(target_id, &sense_key);

            if (sense_key == UNIT_ATTENTION)
            {
                uint8_t inquiry[36];
                dbgmsg("Target ", target_id, " reports UNIT_ATTENTION, running INQUIRY");
                scsiInquiry(target_id, inquiry);
            }
            else if (sense_key == NOT_READY)
            {
                dbgmsg("Target ", target_id, " reports NOT_READY, running STARTSTOPUNIT");
                scsiStartStopUnit(target_id, true);
            }
        }
        else
        {
            dbgmsg("Target ", target_id, " TEST UNIT READY response: ", status);
        }
    }

    return false;
}

bool scsiInitiatorResetBusConfig(int target_id)
{
    uint8_t msgOut[] = {0x80, // Identify
        0x01, 0x03, 0x01, 0x00, 0x00, // Disable synchronous mode
        0x01, 0x02, 0x03, 0x00  // 8-bit mode
    };

    g_initiator_state.targetBusWidth[target_id] = 0;

    int status = scsiInitiatorMessage(target_id, msgOut, sizeof(msgOut), nullptr, 0, nullptr);
    return status == 0;
}

#if !defined(PLATFORM_MAX_BUS_WIDTH) || PLATFORM_MAX_BUS_WIDTH == 0
bool scsiInitiatorSetBusWidth(int target_id, int busWidth)
{
    return false;
}
#else
bool scsiInitiatorSetBusWidth(int target_id, int busWidth)
{
    uint8_t msgOut[] = {0x80, // Identify
        0x01, 0x02, 0x03, (uint8_t)busWidth  // Bus width
    };

    uint8_t msgIn[16] = {0};
    size_t msgInLen = 0;

    dbgmsg("---- Negotiating bus width = ", (uint8_t)busWidth);
    int status = scsiInitiatorMessage(target_id, msgOut, sizeof(msgOut), msgIn, sizeof(msgIn), &msgInLen);
    if (status != 0)
    {
        scsiInitiatorResetBusConfig(target_id);
        return false;
    }

    // Parse response message
    int agreedMode = -1;
    size_t parsed = 0;
    while (parsed < msgInLen)
    {
        uint8_t msgByte = msgIn[parsed++];
        if (msgByte == 0x01)
        {
            // Extended message
            uint8_t extLen = msgIn[parsed++];
            uint8_t *extMsg = &msgIn[parsed];
            parsed += extLen;

            if (extMsg[0] == 0x03)
            {
                dbgmsg("-- Target bus width response: ", extMsg[1]);
                agreedMode = extMsg[1];
            }
        }
        else if ((msgByte & 0xF0) == 0x20)
        {
            // Two-byte message, ignore
            parsed++;
        }
    }

    if (agreedMode < 0)
    {
        logmsg("-- Target did not respond to bus width negotiation, reverting to 8-bit");
        scsiInitiatorResetBusConfig(target_id);
        return false;
    }
    else if (agreedMode == busWidth)
    {
        dbgmsg("-- Negotiated bus width ", 8 << busWidth, " bits, testing with Inquiry command");
        g_initiator_state.targetBusWidth[target_id] = busWidth;
        uint8_t inquiryData[36];
        if (!scsiInquiry(target_id, inquiryData))
        {
            logmsg("-- Bus width test failed, reverting to 8-bit");
            scsiInitiatorResetBusConfig(target_id);
            return false;
        }
        else
        {
            logmsg("-- Successfully negotiated ", 8 << busWidth, " bit bus mode");
            return true;
        }
    }
    else
    {
        logmsg("-- Target refused wide bus request, reverting to 8-bit");
        scsiInitiatorResetBusConfig(target_id);
        return false;
    }
}
#endif

// This uses callbacks to run SD and SCSI transfers in parallel
static struct {
    uint32_t bytes_sd; // Number of bytes that have been transferred on SD card side
    uint32_t bytes_sd_scheduled; // Number of bytes scheduled for transfer on SD card side
    uint32_t bytes_scsi; // Number of bytes that have been scheduled for transfer on SCSI side
    uint32_t bytes_scsi_done; // Number of bytes that have been transferred on SCSI side

    uint32_t bytes_per_sector;
    bool all_ok;
} g_initiator_transfer;

static void initiatorReadSDCallback(uint32_t bytes_complete)
{
    if (g_initiator_transfer.bytes_scsi_done < g_initiator_transfer.bytes_scsi)
    {
        // How many bytes remaining in the transfer?
        uint32_t remain = g_initiator_transfer.bytes_scsi - g_initiator_transfer.bytes_scsi_done;
        uint32_t len = remain;

        // Limit maximum amount of data transferred at one go, to give enough callbacks to SD driver.
        // Select the limit based on total bytes in the transfer.
        // Transfer size is reduced towards the end of transfer to reduce the dead time between
        // end of SCSI transfer and the SD write completing.
        uint32_t limit = g_initiator_transfer.bytes_scsi / 8;
        uint32_t bytesPerSector = g_initiator_transfer.bytes_per_sector;
        if (limit < PLATFORM_OPTIMAL_MIN_SD_WRITE_SIZE) limit = PLATFORM_OPTIMAL_MIN_SD_WRITE_SIZE;
        if (limit > PLATFORM_OPTIMAL_MAX_SD_WRITE_SIZE) limit = PLATFORM_OPTIMAL_MAX_SD_WRITE_SIZE;
        if (limit > len) limit = PLATFORM_OPTIMAL_LAST_SD_WRITE_SIZE;
        if (limit < bytesPerSector) limit = bytesPerSector;

        if (len > limit)
        {
            len = limit;
        }

        // Split read so that it doesn't wrap around buffer edge
        uint32_t bufsize = sizeof(scsiDev.data);
        uint32_t start = (g_initiator_transfer.bytes_scsi_done % bufsize);
        if (start + len > bufsize)
            len = bufsize - start;

        // Don't overwrite data that has not yet been written to SD card
        uint32_t sd_ready_cnt = g_initiator_transfer.bytes_sd + bytes_complete;
        if (g_initiator_transfer.bytes_scsi_done + len > sd_ready_cnt + bufsize)
            len = sd_ready_cnt + bufsize - g_initiator_transfer.bytes_scsi_done;

        if (sd_ready_cnt == g_initiator_transfer.bytes_sd_scheduled &&
            g_initiator_transfer.bytes_sd_scheduled + bytesPerSector <= g_initiator_transfer.bytes_scsi_done)
        {
            // Current SD transfer is complete, it is better we return now and offer a chance for the next
            // transfer to begin.
            return;
        }

        // Keep transfers a multiple of sector size.
        if (remain >= bytesPerSector && len % bytesPerSector != 0)
        {
            len -= len % bytesPerSector;
        }

        if (len == 0)
            return;

        // dbgmsg("SCSI read ", (int)start, " + ", (int)len, ", sd ready cnt ", (int)sd_ready_cnt, " ", (int)bytes_complete, ", scsi done ", (int)g_initiator_transfer.bytes_scsi_done);
        scsiHostSetBusWidth(g_initiator_state.targetBusWidth[g_initiator_state.target_id]);
        uint32_t rxcount = scsiHostRead(&scsiDev.data[start], len);
        scsiHostSetBusWidth(0);
        if (rxcount != len)
        {
            logmsg("Read failed at byte ", (int)g_initiator_transfer.bytes_scsi_done);
            g_initiator_transfer.all_ok = false;
        }
        g_initiator_transfer.bytes_scsi_done += len;
    }
}

static void scsiInitiatorWriteDataToSd(FsFile &file, bool use_callback)
{
    // Figure out longest continuous block in buffer
    uint32_t bufsize = sizeof(scsiDev.data);
    uint32_t start = g_initiator_transfer.bytes_sd % bufsize;
    uint32_t len = g_initiator_transfer.bytes_scsi_done - g_initiator_transfer.bytes_sd;
    if (start + len > bufsize) len = bufsize - start;


    // Try to do writes in multiples that align to both SCSI sectors and SD card sectors.
    // SD cards use 512-byte sectors, so writes should be 512-byte aligned for performance.
    // LCM(512, 520) = 33280 bytes = 64 SCSI sectors = 65 SD sectors.
    uint32_t lcm_alignment = g_initiator_state.lcm_scsi_vs_sd_sectorsize;
    uint32_t sectorsize = g_initiator_state.sectorsize;
    if (sectorsize == SD_SECTOR_SIZE)
    {
        if (len >= SD_SECTOR_SIZE) {
            len &= ~(SD_SECTOR_SIZE - 1);
        }
    }
    else if (lcm_alignment > 0 && len >= lcm_alignment)
    {
        len -= len % lcm_alignment;
    }
    else if (len >= sectorsize)
    {
        // not enough for LCM alignment, but write complete sectors
        len -= len % sectorsize;
    }

    // Start writing to SD card and simultaneously reading more from SCSI bus
    uint8_t *buf = &scsiDev.data[start];
    // dbgmsg("SD write ", (int)start, " + ", (int)len);

    if (use_callback)
    {
        platform_set_sd_callback(&initiatorReadSDCallback, buf);
    }

    g_initiator_transfer.bytes_sd_scheduled = g_initiator_transfer.bytes_sd + len;
    if (file.write(buf, len) != len)
    {
        logmsg("scsiInitiatorReadDataToFile: SD card write failed");
        g_initiator_transfer.all_ok = false;
    }
    platform_set_sd_callback(NULL, NULL);
    g_initiator_transfer.bytes_sd += len;
}

bool scsiInitiatorReadDataToFile(int target_id, uint32_t start_sector, uint32_t sectorcount, uint32_t sectorsize,
                                 FsFile &file)
{
    int status = -1;

    // Read6 command supports 21 bit LBA - max of 0x1FFFFF
    // ref: https://www.seagate.com/files/staticfiles/support/docs/manual/Interface%20manuals/100293068j.pdf pg 134
    bool fits_read6 = (start_sector < 0x1FFFFF && sectorcount <= 256);
    if (!g_initiator_state.use_read10 && fits_read6)
    {
        // Use READ6 command for compatibility with old SCSI1 drives
        // Note that even with SCSI1 drives we have no choice but to use READ10 if the drive
        // size is larger than 1 GB, as the sector number wouldn't fit in the command.
        uint8_t command[6] = {0x08,
            (uint8_t)(start_sector >> 16),
            (uint8_t)(start_sector >> 8),
            (uint8_t)start_sector,
            (uint8_t)sectorcount,
            0x00
        };

        // Start executing command, return in data phase
        status = scsiInitiatorRunCommand(target_id, command, sizeof(command), NULL, 0, NULL, 0, true);
    }
    else
    {
        // Use READ10 command for larger number of blocks
        uint8_t command[10] = {0x28, 0x00,
            (uint8_t)(start_sector >> 24), (uint8_t)(start_sector >> 16),
            (uint8_t)(start_sector >> 8), (uint8_t)start_sector,
            0x00,
            (uint8_t)(sectorcount >> 8), (uint8_t)(sectorcount),
            0x00
        };

        // Start executing command, return in data phase
        status = scsiInitiatorRunCommand(target_id, command, sizeof(command), NULL, 0, NULL, 0, true);
    }


    if (status != 0)
    {
        uint8_t sense_key;
        scsiRequestSense(target_id, &sense_key);

        scsiLogInitiatorCommandFailure("scsiInitiatorReadDataToFile command phase", target_id, status, sense_key);
        scsiHostPhyRelease();
        return false;
    }

    SCSI_PHASE phase;

    g_initiator_transfer.bytes_scsi = sectorcount * sectorsize;
    g_initiator_transfer.bytes_per_sector = sectorsize;
    g_initiator_transfer.bytes_sd = 0;
    g_initiator_transfer.bytes_sd_scheduled = 0;
    g_initiator_transfer.bytes_scsi_done = 0;
    g_initiator_transfer.all_ok = true;

    uint32_t progress_time = millis();
    uint32_t progress_bytes = 0;
    while (true)
    {
        platform_poll();

        phase = (SCSI_PHASE)scsiHostPhyGetPhase();
        if (phase != DATA_IN && phase != BUS_BUSY)
        {
            break;
        }

        // SASI: a target that aborts a transfer can leave the bus in a state that
        // looks like BUS_BUSY. Do not wait for the watchdog if no data arrives.
        if (g_initiator_state.sasi)
        {
            if (g_initiator_transfer.bytes_scsi_done != progress_bytes)
            {
                progress_bytes = g_initiator_transfer.bytes_scsi_done;
                progress_time = millis();
            }
            else if ((uint32_t)(millis() - progress_time) > 10000)
            {
                // Backstop only: scsi_accel_host_read() itself gives up after 8 s in SASI mode.
                logmsg("SASI: no data from target for 10 s in phase ", (int)phase, ", aborting transfer");
                scsiHostPhyReset();
                g_initiator_transfer.all_ok = false;
                break;
            }
        }

        // Read next block from SCSI bus if buffer empty
        if (g_initiator_transfer.bytes_sd == g_initiator_transfer.bytes_scsi_done)
        {
            initiatorReadSDCallback(0);
        }
        else
        {
            // Write data to SD card and simultaneously read more from SCSI
            scsiInitiatorUpdateLed();
            scsiInitiatorWriteDataToSd(file, true);
        }
    }

    // Write any remaining buffered data
    while (g_initiator_transfer.bytes_sd < g_initiator_transfer.bytes_scsi_done)
    {
        platform_poll();
        scsiInitiatorWriteDataToSd(file, false);
    }

    if (g_initiator_transfer.bytes_sd != g_initiator_transfer.bytes_scsi)
    {
        logmsg("SCSI read from sector ", (int)start_sector, " was incomplete: expected ",
             (int)g_initiator_transfer.bytes_scsi, " got ", (int)g_initiator_transfer.bytes_sd, " bytes");
        g_initiator_transfer.all_ok = false;
    }

    uint32_t status_wait_start = millis();
    bool status_wait_timeout = false;
    while ((phase = (SCSI_PHASE)scsiHostPhyGetPhase()) != BUS_FREE)
    {
        platform_poll();

        // SASI targets may leave the bus in an unexpected phase after an aborted
        // data transfer: do not wait for the watchdog, reset the bus instead.
        if (g_initiator_state.sasi && (uint32_t)(millis() - status_wait_start) > 2000)
        {
            logmsg("SASI: target stuck in phase ", (int)phase, " after data transfer, resetting bus");
            scsiHostPhyReset();
            g_initiator_transfer.all_ok = false;
            status_wait_timeout = true;
            break;
        }

        if (phase == MESSAGE_IN)
        {
            uint8_t msg = 0;
            scsiHostRead(&msg, 1);

            if (msg == MSG_COMMAND_COMPLETE)
            {
                break;
            }
        }
        else if (phase == MESSAGE_OUT)
        {
            uint8_t identify_msg = 0x80;
            scsiHostWrite(&identify_msg, 1);
        }
        else if (phase == STATUS)
        {
            uint8_t tmp = 0;
            scsiHostRead(&tmp, 1);
            status = tmp;
            dbgmsg("------ STATUS: ", tmp);
        }
    }

    if (status_wait_timeout)
    {
        return false;
    }

    scsiHostWaitBusFree();

    if (!g_initiator_transfer.all_ok)
    {
        dbgmsg("scsiInitiatorReadDataToFile: Incomplete transfer");
        return false;
    }
    else if (status == 2)
    {
        uint8_t sense_key;
        scsiRequestSense(target_id, &sense_key);

        if (sense_key == RECOVERED_ERROR)
        {
            if (g_initiator_state.sasi)
            {
                logmsg("SASI: correctable data error (ECC) in ", (int)sectorcount, " sector(s) from ",
                       (int)start_sector, ", data accepted");
            }
            else
            {
                dbgmsg("scsiInitiatorReadDataToFile: RECOVERED_ERROR at ", (int)start_sector);
            }
            return true;
        }
        else if (sense_key == UNIT_ATTENTION)
        {
            dbgmsg("scsiInitiatorReadDataToFile: UNIT_ATTENTION");
            return true;
        }
        else
        {
            scsiLogInitiatorCommandFailure("scsiInitiatorReadDataToFile data phase", target_id, status, sense_key);
            return false;
        }
    }
    else
    {
        return status == 0;
    }
}


#endif
