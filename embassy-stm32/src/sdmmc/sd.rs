use core::default::Default;
use core::ops::{Deref, DerefMut, RangeInclusive};

use sdio_host::common_cmd::R1;
use sdio_host::emmc::{EMMC, ExtCSD};
use sdio_host::emmc_cmd::AccessMode;
use sdio_host::sd::{BusWidth, CIC, CID, CSD, CardCapacity, CardStatus, CurrentState, OCR, RCA, SCR, SD, SDStatus};
use sdio_host::{common_cmd, emmc_cmd, sd_cmd};

use crate::sdmmc::{
    BlockSize, DatapathMode, Error, Sdmmc, Signalling, aligned_mut, aligned_ref, block_size, bus_width_vals,
    slice8_mut, slice8_ref,
};
use crate::time::{Hertz, mhz};

// Used as the length of an Ext section in the General Information Memory Section. While the length is variable,
// according to the specification, this is long enough to include Register Set Address 1. Since we only currently
// support extension registers that have only one register set, this is long enough for all our needs.
const EXT_REGISTER_LENGTH: usize = 48;

/// Aligned data block for SDMMC transfers.
///
/// This is a 512-byte array, aligned to 4 bytes to satisfy DMA requirements.
#[repr(align(4))]
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataBlock(pub [u32; 128]);

impl DataBlock {
    /// Create a new DataBlock
    pub const fn new() -> Self {
        DataBlock([0u32; 128])
    }
}

impl Deref for DataBlock {
    type Target = [u8; 512];

    fn deref(&self) -> &Self::Target {
        unwrap!(slice8_ref(&self.0[..]).try_into())
    }
}

impl DerefMut for DataBlock {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unwrap!(slice8_mut(&mut self.0[..]).try_into())
    }
}

/// Command Block buffer for SDMMC command transfers.
///
/// This is a 16-word array, exposed so that DMA commpatible memory can be used if required.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CmdBlock(pub [u32; 16]);

impl CmdBlock {
    /// Creates a new instance of CmdBlock
    pub const fn new() -> Self {
        Self([0u32; 16])
    }
}

impl Deref for CmdBlock {
    type Target = [u32; 16];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for CmdBlock {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Represents either an SD or EMMC card
pub trait Addressable: Sized + Clone {
    /// Associated type
    type Ext;

    /// Get this peripheral's address on the SDMMC bus
    fn get_address(&self) -> u16;

    /// Is this a standard or high capacity peripheral?
    fn get_capacity(&self) -> CardCapacity;

    /// Size in bytes
    fn size(&self) -> u64;
}

/// Storage Device
pub struct StorageDevice<'a, 'b, T: Addressable> {
    info: T,
    /// Inner member
    pub sdmmc: &'a mut Sdmmc<'b>,
}

/// Card Storage Device
impl<'a, 'b> StorageDevice<'a, 'b, Card> {
    /// Create a new SD card
    pub async fn new_sd_card(sdmmc: &'a mut Sdmmc<'b>, cmd_block: &mut CmdBlock, freq: Hertz) -> Result<Self, Error> {
        let mut s = Self {
            info: Card::default(),
            sdmmc,
        };

        s.acquire(cmd_block, freq).await?;

        Ok(s)
    }

    /// Initializes the card into a known state (or at least tries to).
    async fn acquire(&mut self, cmd_block: &mut CmdBlock, freq: Hertz) -> Result<(), Error> {
        let _scoped_block_stop = self.sdmmc.info.rcc.block_stop();
        let regs = self.sdmmc.info.regs;

        // Get the bus width configured in the Sdmmc peripheral
        let configured_bus_width = match self.sdmmc.bus_width() {
            BusWidth::Eight => return Err(Error::BusWidth),
            bus_width => bus_width,
        };

        // While the SD/SDIO card or eMMC is in identification mode,
        // the SDMMC_CK frequency must be no more than 400 kHz.
        self.sdmmc.init_idle()?;

        // Check if cards supports CMD8 (with pattern)
        self.sdmmc.cmd(sd_cmd::send_if_cond(1, 0xAA), true, false)?;
        let cic = CIC::from(regs.respr(0).read().cardstatus());

        if cic.pattern() != 0xAA {
            return Err(Error::UnsupportedCardVersion);
        }

        if cic.voltage_accepted() & 1 == 0 {
            return Err(Error::UnsupportedVoltage);
        }

        let ocr = loop {
            // Signal that next command is a app command
            self.sdmmc.cmd(common_cmd::app_cmd(0), true, false)?; // CMD55

            // 3.2-3.3V
            let voltage_window = 1 << 5;
            // Initialize card

            let ocr: OCR<SD> = self
                .sdmmc
                .cmd(sd_cmd::sd_send_op_cond(true, false, true, voltage_window), false, false)?
                .into();

            if !ocr.is_busy() {
                // Power up done
                break ocr;
            }
        };

        if ocr.high_capacity() {
            // Card is SDHC or SDXC or SDUC
            self.info.card_type = CardCapacity::HighCapacity;
        } else {
            self.info.card_type = CardCapacity::StandardCapacity;
        }
        self.info.ocr = ocr;

        self.info.cid = self.sdmmc.get_cid()?.into();
        let rca: RCA<SD> = self.sdmmc.cmd(sd_cmd::send_relative_address(), true, false)?.into();
        self.info.rca = rca.address();
        self.info.csd = self.sdmmc.get_csd(self.info.get_address())?.into();
        self.sdmmc.select_card(Some(self.info.get_address()))?;
        self.info.scr = self.get_scr(cmd_block).await?;

        // Select bus width based on Sdmmc configuration and card capability
        // Use 4-bit only if both the peripheral is configured for it AND the card supports it
        let (bus_width, acmd_arg) = match configured_bus_width {
            BusWidth::Four if self.info.scr.bus_width_four() => (BusWidth::Four, 2),
            _ => (BusWidth::One, 0),
        };

        self.sdmmc.cmd(common_cmd::app_cmd(self.info.rca), true, false)?;
        self.sdmmc.cmd(sd_cmd::cmd6(acmd_arg), true, false)?;

        self.sdmmc.clkcr_set_clkdiv(freq.clamp(mhz(0), mhz(25)), bus_width)?;

        // Read status
        self.info.status = self.read_sd_status(cmd_block).await?;

        if freq > mhz(25) {
            // Switch to SDR25
            let signalling = self.switch_signalling_mode(cmd_block, Signalling::SDR25).await?;

            if signalling == Signalling::SDR25 {
                // Set final clock frequency
                self.sdmmc.clkcr_set_clkdiv(freq, bus_width)?;

                let status: CardStatus<SD> = self.sdmmc.read_status(self.info.rca)?.into();
                if status.state() != CurrentState::Transfer {
                    return Err(Error::SignalingSwitchFailed);
                }
            }

            // Read status after signalling change
            self.read_sd_status(cmd_block).await?;
        }

        // TODO: Create a better abstraction for this
        let cmd48_support = (self.info.scr.0 & (1 << 34)) != 0;
        if cmd48_support {
            self.read_ext_registers().await?;
        }

        Ok(())
    }

    /// Switch mode using CMD6.
    ///
    /// Attempt to set a new signalling mode. The selected
    /// signalling mode is returned. Expects the current clock
    /// frequency to be > 12.5MHz.
    ///
    /// SD only.
    async fn switch_signalling_mode(
        &self,
        cmd_block: &mut CmdBlock,
        signalling: Signalling,
    ) -> Result<Signalling, Error> {
        // NB PLSS v7_10 4.3.10.4: "the use of SET_BLK_LEN command is not
        // necessary"

        let set_function = 0x8000_0000
            | match signalling {
                // See PLSS v7_10 Table 4-11
                Signalling::DDR50 => 0xFF_FF04,
                Signalling::SDR104 => 0xFF_1F03,
                Signalling::SDR50 => 0xFF_1F02,
                Signalling::SDR25 => 0xFF_FF01,
                Signalling::SDR12 => 0xFF_FF00,
            };

        let buffer = &mut aligned_mut(&mut cmd_block.0)[..64];
        let mode = DatapathMode::Block(block_size(size_of_val(buffer)));
        let transfer = self.sdmmc.prepare_datapath_read(buffer, mode);

        self.sdmmc.cmd(sd_cmd::cmd6(set_function), true, true)?; // CMD6

        self.sdmmc.complete_datapath_transfer(transfer, true).await?;

        // Host is allowed to use the new functions at least 8
        // clocks after the end of the switch command
        // transaction. We know the current clock period is < 80ns,
        // so a total delay of 640ns is required here
        for _ in 0..300 {
            cortex_m::asm::nop();
        }

        // Function Selection of Function Group 1
        let selection = (u32::from_be(cmd_block[4]) >> 24) & 0xF;

        match selection {
            0 => Ok(Signalling::SDR12),
            1 => Ok(Signalling::SDR25),
            2 => Ok(Signalling::SDR50),
            3 => Ok(Signalling::SDR104),
            4 => Ok(Signalling::DDR50),
            _ => Err(Error::UnsupportedCardType),
        }
    }

    /// Reads the SCR register.
    ///
    /// SD only.
    async fn get_scr(&self, cmd_block: &mut CmdBlock) -> Result<SCR, Error> {
        // Read the 64-bit SCR register
        self.sdmmc.cmd(common_cmd::set_block_length(8), true, false)?; // CMD16
        self.sdmmc.cmd(common_cmd::app_cmd(self.info.rca), true, false)?;

        let scr = &mut cmd_block.0[..2];

        // Arm `OnDrop` after the buffer, so it will be dropped first

        let transfer = self
            .sdmmc
            .prepare_datapath_read(aligned_mut(scr), DatapathMode::Block(BlockSize::Size8));
        self.sdmmc.cmd(sd_cmd::send_scr(), true, true)?;

        self.sdmmc.complete_datapath_transfer(transfer, true).await?;

        Ok(SCR(u64::from_be_bytes(unwrap!(slice8_mut(scr).try_into()))))
    }

    /// Reads the SD Status (ACMD13)
    ///
    /// SD only.
    async fn read_sd_status(&self, cmd_block: &mut CmdBlock) -> Result<SDStatus, Error> {
        let rca = self.info.rca;
        let buffer = &mut aligned_mut(&mut cmd_block.0)[..64];

        self.sdmmc
            .cmd(common_cmd::set_block_length(size_of_val(buffer) as u32), true, false)?; // CMD16
        self.sdmmc.cmd(common_cmd::app_cmd(rca), true, false)?; // APP

        let mode = DatapathMode::Block(block_size(size_of_val(buffer)));
        let transfer = self.sdmmc.prepare_datapath_read(buffer, mode);

        self.sdmmc.cmd(sd_cmd::sd_status(), true, true)?;
        self.sdmmc.complete_datapath_transfer(transfer, true).await?;

        for word in cmd_block.iter_mut() {
            *word = u32::from_be(*word);
        }

        Ok(cmd_block.0.into())
    }

    /// Erase one (or more) SDMMC blocks
    pub fn erase_blocks(&mut self, groups: RangeInclusive<u32>) -> Result<(), Error> {
        self.sdmmc
            .cmd(common_cmd::cmd::<R1>(32, *groups.start()), true, false)?;
        self.sdmmc.cmd(common_cmd::cmd::<R1>(33, *groups.end()), true, false)?;
        self.sdmmc.cmd(common_cmd::erase(), true, false)?;
        self.poll_ready_for_data(None)
    }

    /// Build the argument for CMD48/49 (Read/Write Ext Register)
    fn make_ext_reg_argument(fno: u8, page: u8, offset: u16, buffer: &DataBlock) -> u32 {
        // Argument Structure:
        // [31] = 0 (MIO Memory)
        // [30:27] = FNO (Function Number)
        // [26] = MW - mask write mode
        // [25:18] = offset address
        // [8:0] = length - 1 (0 is 1 byte)
        //    FNO (Function Number), [26] = MW (Memory Write), [25:9] = Addr (Register Address)
        let length: u32 = (size_of_val(buffer) - 1).try_into().unwrap();
        u32::from(fno) << 27 | u32::from(page) << 18 | u32::from(offset) << 9 | length
    }

    async fn read_ext_reg(&mut self, fno: u8, page: u8, offset: u16, buffer: &mut DataBlock) -> Result<(), Error> {
        let argument = Self::make_ext_reg_argument(fno, page, offset, buffer);
        let mode = DatapathMode::Block(block_size(size_of_val(buffer)));
        let transfer = self.sdmmc.prepare_datapath_read(aligned_mut(&mut buffer.0), mode);

        self.sdmmc.cmd(common_cmd::cmd::<R1>(48, argument), true, true)?;
        self.sdmmc.complete_datapath_transfer(transfer, true).await
    }

    // Read the General Information for Memory section (section "General Information", Physical Layer Simplified Specification, v9.10).
    // This section contains pointers to additional extension registers with function-specific information. Function specific information
    // is populated in `info` if it exists.
    async fn read_ext_registers(&mut self) -> Result<(), Error> {
        let mut data_block = DataBlock([0u32; 128]);
        self.read_ext_reg(0, 0, 0, &mut data_block).await?;
        // The first 16 bytes of the General Information is header.
        let mut offset: usize = 16;
        while offset < (size_of_val(&data_block.0) - EXT_REGISTER_LENGTH) {
            let register = &(*data_block)[offset..offset + EXT_REGISTER_LENGTH];
            let (next_offset, register) = self.parse_ext_reg(register).await?;
            offset = next_offset;
            if let ExtensionRegister::PowerManagement(power_ext) = register {
                self.info.power_ext = Some(power_ext);
            }
        }
        Ok(())
    }

    async fn parse_ext_reg(&mut self, register_set: &[u8]) -> Result<(usize, ExtensionRegister), Error> {
        assert!(register_set.len() >= EXT_REGISTER_LENGTH);
        let sfc = u16::from_be_bytes(register_set[0..2].try_into().unwrap());
        let next_extension_address = u16::from_be_bytes(register_set[40..42].try_into().unwrap());
        let number_of_registers: u8 = register_set[42];
        if number_of_registers != 1 {
            return Ok((next_extension_address as usize, ExtensionRegister::Unsupported));
        }

        let register_address = u32::from_be_bytes(register_set[44..].try_into().unwrap());
        let offset: u16 = (register_address & 0x1FF).try_into().unwrap();
        let page: u8 = (register_address >> 9 & 0xFF).try_into().unwrap();
        let fno: u8 = (register_address >> 18 & 0xF).try_into().unwrap();

        let register = if sfc & 0x1 != 0 {
            let mut data_block = DataBlock([0u32; 128]);
            self.read_ext_reg(fno, page, offset, &mut data_block).await?;
            let supports_power_off_notification = data_block[1] & (1 << 4) != 0;
            ExtensionRegister::PowerManagement(PowerExtensionRegister {
                supports_power_off_notification,
                fno,
                page,
                offset,
            })
        } else {
            ExtensionRegister::Unsupported
        };
        Ok((next_extension_address as usize, register))
    }

    async fn write_ext_reg(&mut self, fno: u8, page: u8, offset: u16, buffer: &DataBlock) -> Result<(), Error> {
        self.sdmmc
            .cmd(common_cmd::set_block_length(size_of::<DataBlock>() as u32), true, false)?; // CMD16
        let argument = Self::make_ext_reg_argument(fno, page, offset, buffer);

        // sdmmc_v1 uses different cmd/dma order than v2, but only for writes
        #[cfg(sdmmc_v1)]
        self.sdmmc.cmd(common_cmd::cmd::<R1>(49, argument), true, false)?;

        let transfer = self.sdmmc.prepare_datapath_write(
            aligned_ref(&buffer.0),
            DatapathMode::Block(block_size(size_of::<DataBlock>())),
        );

        #[cfg(sdmmc_v2)]
        self.sdmmc.cmd(common_cmd::cmd::<R1>(49, argument), true, false)?;

        self.sdmmc.complete_datapath_transfer(transfer, true).await?;

        // TODO: Make this configurable
        const TIMEOUT: u32 = 0x00FF_FFFF;
        self.poll_ready_for_data(Some(TIMEOUT))
    }

    /// Send a power-off notification to the card. On SD cards supporting v4.00+, send this notification to
    /// the card to indicate that shutdown of card power is imminent. Does not return until the card indicates
    /// it is safe to remove power.
    #[cfg(feature = "time")]
    pub async fn power_off_notify(&mut self) -> Result<(), Error> {
        use embassy_time::{Duration, Instant};
        let Some(PowerExtensionRegister {
            fno,
            page,
            offset,
            supports_power_off_notification,
        }) = self.info.power_ext
        else {
            return Err(Error::UnsupportedOperation);
        };

        if !supports_power_off_notification {
            return Err(Error::UnsupportedOperation);
        }

        let mut buffer = DataBlock([0u32; 128]);
        // Set the POFN bit in the Power Management Setting Register.
        buffer[0] = 0x1;
        self.write_ext_reg(fno, page, offset + 2, &buffer).await?;

        // Wait for the POFR bit in the Power Management Status Register to be set to 1.
        const TIMEOUT: Duration = Duration::from_secs(1);
        let start = Instant::now();
        while Instant::now().duration_since(start) < TIMEOUT {
            self.read_ext_reg(fno, page, offset + 1, &mut buffer).await?;
            if (buffer[0] & 0x1) != 0 {
                return Ok(());
            }
        }
        Err(Error::SoftwareTimeout)
    }
}

/// Emmc storage device
impl<'a, 'b> StorageDevice<'a, 'b, Emmc> {
    const POWER_OFF_NOTIFICATION_EXT_CSD_INDEX: u8 = 34;

    /// Create a new EMMC card
    pub async fn new_emmc(sdmmc: &'a mut Sdmmc<'b>, cmd_block: &mut CmdBlock, freq: Hertz) -> Result<Self, Error> {
        let mut s = Self {
            info: Emmc::default(),
            sdmmc,
        };

        s.acquire(cmd_block, freq).await?;

        Ok(s)
    }

    async fn acquire(&mut self, _cmd_block: &mut CmdBlock, freq: Hertz) -> Result<(), Error> {
        let _scoped_block_stop = self.sdmmc.info.rcc.block_stop();
        let regs = self.sdmmc.info.regs;

        let bus_width = self.sdmmc.bus_width();

        // While the SD/SDIO card or eMMC is in identification mode,
        // the SDMMC_CK frequency must be no more than 400 kHz.
        self.sdmmc.init_idle()?;

        let ocr = loop {
            let high_voltage = 0b0 << 7;
            let access_mode = 0b10 << 29;
            let op_cond = high_voltage | access_mode | 0b1_1111_1111 << 15;
            // Initialize card
            match self.sdmmc.cmd(emmc_cmd::send_op_cond(op_cond), true, false) {
                Ok(_) => (),
                Err(Error::Crc) => (),
                Err(err) => return Err(err),
            }
            let ocr: OCR<EMMC> = regs.respr(0).read().cardstatus().into();
            if !ocr.is_busy() {
                // Power up done
                break ocr;
            }
        };

        self.info.capacity = if ocr.access_mode() == 0b10 {
            // Card is SDHC or SDXC or SDUC
            CardCapacity::HighCapacity
        } else {
            CardCapacity::StandardCapacity
        };
        self.info.ocr = ocr;
        self.info.cid = self.sdmmc.get_cid()?.into();
        self.info.rca = 1u16.into();

        self.sdmmc
            .cmd(emmc_cmd::assign_relative_address(self.info.rca), true, false)?;

        self.info.csd = self.sdmmc.get_csd(self.info.get_address())?.into();
        self.sdmmc.select_card(Some(self.info.get_address()))?;

        let (widbus, _) = bus_width_vals(bus_width);

        // Write bus width to ExtCSD byte 183
        self.sdmmc.cmd(
            emmc_cmd::modify_ext_csd(emmc_cmd::AccessMode::WriteByte, 183, widbus),
            true,
            false,
        )?;

        // Wait for ready after R1b response
        loop {
            let status: CardStatus<EMMC> = self.sdmmc.read_status(self.info.rca)?.into();
            if status.ready_for_data() {
                break;
            }
        }

        // Enable the power off notification functionality. 
        self.sdmmc.cmd(
            emmc_cmd::modify_ext_csd(emmc_cmd::AccessMode::WriteByte, Self::POWER_OFF_NOTIFICATION_EXT_CSD_INDEX, 1),
            true,
            false,
        )?;

        loop {
            let status: CardStatus<EMMC> = self.sdmmc.read_status(self.info.rca)?.into();
            if status.ready_for_data() {
                break;
            }
        }

        self.sdmmc.clkcr_set_clkdiv(freq.clamp(mhz(0), mhz(25)), bus_width)?;
        self.info.ext_csd = self.read_ext_csd().await?;

        Ok(())
    }

    /// Gets the EXT_CSD register.
    ///
    /// eMMC only.
    async fn read_ext_csd(&self) -> Result<ExtCSD, Error> {
        // Note: cmd_block can't be used because ExtCSD is too long to fit.
        let mut data_block = DataBlock::new();

        self.sdmmc
            .cmd(common_cmd::set_block_length(size_of::<DataBlock>() as u32), true, false)
            .unwrap(); // CMD16

        let transfer = self.sdmmc.prepare_datapath_read(
            aligned_mut(&mut data_block.0),
            DatapathMode::Block(block_size(size_of::<DataBlock>())),
        );
        self.sdmmc.cmd(emmc_cmd::send_ext_csd(), true, true)?;

        self.sdmmc.complete_datapath_transfer(transfer, true).await?;

        Ok(data_block.0.into())
    }

    /// Erase one (or more) eMMC groups
    pub fn erase_groups(&mut self, groups: RangeInclusive<u32>) -> Result<(), Error> {
        self.sdmmc
            .cmd(emmc_cmd::erase_group_start(*groups.start()), true, false)?;
        self.sdmmc.cmd(emmc_cmd::erase_group_end(*groups.end()), true, false)?;
        self.sdmmc.cmd(common_cmd::erase(), true, false)?;
        self.poll_ready_for_data(None)
    }

    /// Send a short power off notification to the card and wait until the card indicates it is ready for shutdown.
    #[cfg(feature = "time")]
    pub async fn short_power_off_notify(&mut self) -> Result<(), Error> {
        use embassy_time::{Duration, Instant};
        const DEFAULT_POWER_OFF_TIMEOUT: Duration = Duration::from_millis(500);
        let timeout = if self.info.ext_csd.csd_structure_version() >= 6 {
            // Byte 248 in the CSD is the GENERIC_CMD6_TIMEOUT field, in units of 10ms.
            let millis = ((self.info.ext_csd.inner[62] >> 24) & 0xFF) * 10;
            Duration::from_millis(millis.into())
        } else {
            DEFAULT_POWER_OFF_TIMEOUT
        };

        const POWER_OFF_SHORT: u8 = 2;

        // Always send POWER_OFF_SHORT
        self.sdmmc.cmd(
            emmc_cmd::modify_ext_csd(AccessMode::WriteByte, Self::POWER_OFF_NOTIFICATION_EXT_CSD_INDEX, POWER_OFF_SHORT),
            true,
            false,
        )?;

        let start = Instant::now();
        while Instant::now().duration_since(start) < timeout {
            let status: CardStatus<Emmc> = self.sdmmc.read_status(self.info.get_address())?.into();
            if status.ready_for_data() {
                return Ok(());
            }
        }
        Err(Error::SoftwareTimeout)
    }

    /// Send a long power off notification to the card and wait until the card indicates it is ready for shutdown.
    #[cfg(feature = "time")]
    pub async fn long_power_off_notify(&mut self) -> Result<(), Error> {
        use embassy_time::{Duration, Instant};
        const DEFAULT_POWER_OFF_TIMEOUT: Duration = Duration::from_millis(500);
        let timeout = if self.info.ext_csd.csd_structure_version() >= 6 {
            // Byte 247 in the CSD is the POWER_OFF_LONG_TIME field, in units of 10ms.
            let millis = ((self.info.ext_csd.inner[61] >> 24) & 0xFF) * 10;
            Duration::from_millis(millis.into())
        } else {
            DEFAULT_POWER_OFF_TIMEOUT
        };

        const POWER_OFF_LONG: u8 = 3;

        self.sdmmc.cmd(
            emmc_cmd::modify_ext_csd(AccessMode::WriteByte, Self::POWER_OFF_NOTIFICATION_EXT_CSD_INDEX, POWER_OFF_LONG),
            true,
            false,
        )?;

        let start = Instant::now();
        while Instant::now().duration_since(start) < timeout {
            let status: CardStatus<Emmc> = self.sdmmc.read_status(self.info.get_address())?.into();
            let ext_csd = self.read_ext_csd().await?;
            let power_off_status = (ext_csd.inner[61] >> 24) & 0xFF;
            if status.ready_for_data() && power_off_status == POWER_OFF_LONG {
                return Ok(());
            }
        }
        Err(Error::SoftwareTimeout)
    }
}

/// Card or Emmc storage device
impl<'a, 'b, A: Addressable> StorageDevice<'a, 'b, A> {
    /// Write a block
    pub fn card(&self) -> A {
        self.info.clone()
    }

    /// Read a data block.
    #[inline]
    pub async fn read_block(&mut self, block_idx: u32, data_block: &mut DataBlock) -> Result<(), Error> {
        let _scoped_block_stop = self.sdmmc.info.rcc.block_stop();
        let card_capacity = self.info.get_capacity();

        // Always read 1 block of 512 bytes
        // SDSC cards are byte addressed hence the blockaddress is in multiples of 512 bytes
        let address = match card_capacity {
            CardCapacity::StandardCapacity => block_idx * size_of::<DataBlock>() as u32,
            _ => block_idx,
        };
        self.sdmmc
            .cmd(common_cmd::set_block_length(size_of::<DataBlock>() as u32), true, false)?; // CMD16

        let transfer = self.sdmmc.prepare_datapath_read(
            aligned_mut(&mut data_block.0),
            DatapathMode::Block(block_size(size_of::<DataBlock>())),
        );
        self.sdmmc.cmd(common_cmd::read_single_block(address), true, true)?;

        self.sdmmc.complete_datapath_transfer(transfer, true).await?;

        Ok(())
    }

    /// Read multiple data blocks.
    #[inline]
    pub async fn read_blocks(&mut self, block_idx: u32, blocks: &mut [DataBlock]) -> Result<(), Error> {
        let _scoped_block_stop = self.sdmmc.info.rcc.block_stop();
        let card_capacity = self.info.get_capacity();

        // NOTE(unsafe) reinterpret buffer as &mut [u32]
        let buffer = unsafe {
            core::slice::from_raw_parts_mut(
                blocks.as_mut_ptr() as *mut u32,
                blocks.len() * size_of::<DataBlock>() / size_of::<u32>(),
            )
        };

        // Always read 1 block of 512 bytes
        // SDSC cards are byte addressed hence the blockaddress is in multiples of 512 bytes
        let address = match card_capacity {
            CardCapacity::StandardCapacity => block_idx * size_of::<DataBlock>() as u32,
            _ => block_idx,
        };
        self.sdmmc
            .cmd(common_cmd::set_block_length(size_of::<DataBlock>() as u32), true, false)?; // CMD16

        let transfer = self.sdmmc.prepare_datapath_read(
            aligned_mut(buffer),
            DatapathMode::Block(block_size(size_of::<DataBlock>())),
        );
        self.sdmmc.cmd(common_cmd::read_multiple_blocks(address), true, true)?;

        self.sdmmc.complete_datapath_transfer(transfer, false).await?;

        self.sdmmc.cmd(common_cmd::stop_transmission(), true, false)?; // CMD12
        self.sdmmc.clear_interrupt_flags();

        Ok(())
    }

    /// Write a data block.
    pub async fn write_block(&mut self, block_idx: u32, buffer: &DataBlock) -> Result<(), Error>
    where
        CardStatus<A::Ext>: From<u32>,
    {
        let _scoped_block_stop = self.sdmmc.info.rcc.block_stop();

        // Always read 1 block of 512 bytes
        //  cards are byte addressed hence the blockaddress is in multiples of 512 bytes
        let address = match self.info.get_capacity() {
            CardCapacity::StandardCapacity => block_idx * size_of::<DataBlock>() as u32,
            _ => block_idx,
        };
        self.sdmmc
            .cmd(common_cmd::set_block_length(size_of::<DataBlock>() as u32), true, false)?; // CMD16

        // sdmmc_v1 uses different cmd/dma order than v2, but only for writes
        #[cfg(sdmmc_v1)]
        self.sdmmc.cmd(common_cmd::write_single_block(address), true, true)?;

        let transfer = self.sdmmc.prepare_datapath_write(
            aligned_ref(&buffer.0),
            DatapathMode::Block(block_size(size_of::<DataBlock>())),
        );

        #[cfg(sdmmc_v2)]
        self.sdmmc.cmd(common_cmd::write_single_block(address), true, true)?;

        self.sdmmc.complete_datapath_transfer(transfer, true).await?;

        // TODO: Make this configurable
        let timeout: u32 = 0x00FF_FFFF;
        self.poll_ready_for_data(Some(timeout))
    }

    /// Write multiple data blocks.
    pub async fn write_blocks(&mut self, block_idx: u32, blocks: &[DataBlock]) -> Result<(), Error>
    where
        CardStatus<A::Ext>: From<u32>,
    {
        let _scoped_block_stop = self.sdmmc.info.rcc.block_stop();

        // NOTE(unsafe) reinterpret buffer as &[u32]
        let buffer = unsafe {
            core::slice::from_raw_parts(
                blocks.as_ptr() as *const u32,
                blocks.len() * size_of::<DataBlock>() / size_of::<u32>(),
            )
        };
        // Always read 1 block of 512 bytes
        // SDSC cards are byte addressed hence the blockaddress is in multiples of 512 bytes
        let address = match self.info.get_capacity() {
            CardCapacity::StandardCapacity => block_idx * size_of::<DataBlock>() as u32,
            _ => block_idx,
        };

        self.sdmmc
            .cmd(common_cmd::set_block_length(size_of::<DataBlock>() as u32), true, false)?; // CMD16

        #[cfg(sdmmc_v1)]
        self.sdmmc.cmd(common_cmd::write_multiple_blocks(address), true, true)?; // CMD25

        // Setup write command
        let transfer = self.sdmmc.prepare_datapath_write(
            aligned_ref(buffer),
            DatapathMode::Block(block_size(size_of::<DataBlock>())),
        );
        #[cfg(sdmmc_v2)]
        self.sdmmc.cmd(common_cmd::write_multiple_blocks(address), true, true)?; // CMD25

        self.sdmmc.complete_datapath_transfer(transfer, false).await?;

        self.sdmmc.cmd(common_cmd::stop_transmission(), true, false)?; // CMD12
        self.sdmmc.clear_interrupt_flags();

        // TODO: Make this configurable
        let timeout: u32 = 0x00FF_FFFF;
        self.poll_ready_for_data(Some(timeout))
    }

    // TODO: This should be time-based, but the dependency on embassy-time is currently marked optional in Cargo.toml
    fn poll_ready_for_data(&mut self, timeout: Option<u32>) -> Result<(), Error> {
        let infinite = timeout.is_none();
        let mut timeout = timeout.unwrap_or(0);
        while timeout > 0 || infinite {
            let status: CardStatus<A::Ext> = self.sdmmc.read_status(self.info.get_address())?.into();
            if status.ready_for_data() {
                return Ok(());
            }
            timeout = timeout.saturating_sub(1);
        }
        Err(Error::SoftwareTimeout)
    }
}

impl<'a, 'b, A: Addressable> Drop for StorageDevice<'a, 'b, A> {
    fn drop(&mut self) {
        self.sdmmc.on_drop();
    }
}

/// Data provided by the power management function extension register.
#[derive(Clone, Copy, Debug)]
pub struct PowerExtensionRegister {
    // This card supports Power Off Notification
    supports_power_off_notification: bool,
    // Function Number
    fno: u8,
    // Page
    page: u8,
    // Offset
    offset: u16,
}

/// Extension Registers
pub enum ExtensionRegister {
    /// The Power Management Function Extension Register.
    PowerManagement(PowerExtensionRegister),
    /// All other Extension Registers are unsupported at this time.
    Unsupported,
}

#[derive(Clone, Copy, Debug, Default)]
/// SD Card
pub struct Card {
    /// The type of this card
    pub card_type: CardCapacity,
    /// Operation Conditions Register
    pub ocr: OCR<SD>,
    /// Relative Card Address
    pub rca: u16,
    /// Card ID
    pub cid: CID<SD>,
    /// Card Specific Data
    pub csd: CSD<SD>,
    /// SD CARD Configuration Register
    pub scr: SCR,
    /// SD Status
    pub status: SDStatus,
    /// Power Management Function data, if available.
    pub power_ext: Option<PowerExtensionRegister>,
}

impl Addressable for Card {
    type Ext = SD;

    /// Get this peripheral's address on the SDMMC bus
    fn get_address(&self) -> u16 {
        self.rca
    }

    /// Is this a standard or high capacity peripheral?
    fn get_capacity(&self) -> CardCapacity {
        self.card_type
    }

    /// Size in bytes
    fn size(&self) -> u64 {
        u64::from(self.csd.block_count()) * 512
    }
}

#[derive(Clone, Copy, Debug, Default)]
/// eMMC storage
pub struct Emmc {
    /// The capacity of this card
    pub capacity: CardCapacity,
    /// Operation Conditions Register
    pub ocr: OCR<EMMC>,
    /// Relative Card Address
    pub rca: u16,
    /// Card ID
    pub cid: CID<EMMC>,
    /// Card Specific Data
    pub csd: CSD<EMMC>,
    /// Extended Card Specific Data
    pub ext_csd: ExtCSD,
}

impl Addressable for Emmc {
    type Ext = EMMC;

    /// Get this peripheral's address on the SDMMC bus
    fn get_address(&self) -> u16 {
        self.rca
    }

    /// Is this a standard or high capacity peripheral?
    fn get_capacity(&self) -> CardCapacity {
        self.capacity
    }

    /// Size in bytes
    fn size(&self) -> u64 {
        u64::from(self.ext_csd.sector_count()) * 512
    }
}

impl<'d, 'e, A: Addressable> block_device_driver::BlockDevice<512> for StorageDevice<'d, 'e, A> {
    type Error = Error;
    type Align = aligned::A4;

    async fn read(
        &mut self,
        block_address: u32,
        buf: &mut [aligned::Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        // TODO: I think block_address needs to be adjusted by the partition start offset
        if buf.len() == 1 {
            let block = unsafe { &mut *(&mut buf[0] as *mut _ as *mut DataBlock) };
            self.read_block(block_address, block).await?;
        } else {
            let blocks: &mut [DataBlock] =
                unsafe { core::slice::from_raw_parts_mut(buf.as_mut_ptr() as *mut DataBlock, buf.len()) };
            self.read_blocks(block_address, blocks).await?;
        }
        Ok(())
    }

    async fn write(
        &mut self,
        block_address: u32,
        buf: &[aligned::Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        // TODO: I think block_address needs to be adjusted by the partition start offset
        if buf.len() == 1 {
            let block = unsafe { &*(&buf[0] as *const _ as *const DataBlock) };
            self.write_block(block_address, block).await?;
        } else {
            let blocks: &[DataBlock] =
                unsafe { core::slice::from_raw_parts(buf.as_ptr() as *const DataBlock, buf.len()) };
            self.write_blocks(block_address, blocks).await?;
        }
        Ok(())
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        Ok(self.info.size())
    }
}
