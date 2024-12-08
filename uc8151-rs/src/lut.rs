use core::fmt;

/// Dynamic lookup-table code, ported from https://github.com/antirez/uc8151_micropython
///
/// Comments in this file are largely verbatim from the above repo.
/// They were too good to not copy, and explain the functionality of this hardware better
/// than the official documentation.

#[derive(Debug)]
pub(crate) struct VComLut {
    pub lut: [u8; 44],
}

#[derive(Debug)]
pub(crate) struct TransitionLut {
    pub lut: [u8; 42],
}

#[derive(Debug)]
pub(crate) struct CustomLut {
    pub vcom: VComLut,
    pub ww: TransitionLut,
    pub bw: TransitionLut,
    pub wb: TransitionLut,
    pub bb: TransitionLut,
    // pub update_time: u16,
    // pub pll: pll_flags::PllFlags,
}

#[allow(non_camel_case_types)]
enum Lut {
    vc,
    ww,
    bw,
    wb,
    bb,
}

impl CustomLut {
    /// Set a given row in a waveform lookup table.  
    ///
    /// Lookup tables are 6 rows per 7 cols, like in this example:  
    /// ```no_run
    /// 0x40, 0x17, 0x00, 0x00, 0x00, 0x02,  <- step 0  
    /// 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,  <- step 1  
    /// 0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,  <- step 2  
    /// 0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,  <- step 3  
    /// 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  <- step 4  
    /// 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  <- step 5  
    /// 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  <- step 6  
    /// ```
    /// For each step the first byte encodes the 4 patterns, two bits each.  
    /// The next 4 bytes the duration in frames. The Final byte is the repetition number.  
    /// See the top comment of set_waveform_lut() for more info.
    pub fn set_lut_row(&mut self, lut: Lut, row: usize, pat: u8, dur: [u8; 4], rep: u8) {
        if row > 6 {
            panic!("LUTs have 7 total rows (0-6)")
        }
        // Index of byte at start of row `row`
        let row_offset = 6 * row;
        let mut lut = match lut {
            Lut::vc => &mut self.vcom.lut[0..42], // Need to return a subslice to keep all types the same
            Lut::ww => &mut self.ww.lut,
            Lut::bw => &mut self.bw.lut,
            Lut::wb => &mut self.wb.lut,
            Lut::bb => &mut self.bb.lut,
        };
        // Confirm we're not going to index off the end of the slice first
        assert!(row_offset + 5 < lut.len());
        lut[row_offset] = pat;
        lut[row_offset + 1] = dur[0];
        lut[row_offset + 2] = dur[1];
        lut[row_offset + 3] = dur[2];
        lut[row_offset + 4] = dur[3];
        lut[row_offset + 5] = rep;
    }

    /// This function sets the lookup tables used during the display refresh.
    ///
    /// Before reading it, it's a good idea to understand how LUTs are encoded:  
    /// We have a table for each transition possible:
    /// white -> white (WW)
    /// white -> black (WB)
    /// black -> black (BB)
    /// black -> white (BW)
    /// and a final table that controls the VCOM common voltage.
    ///
    /// The update process happens in steps, each 7 rows of each
    /// table tells the display how to set each pixel based on the
    /// transition (WW, WB, BB, BW) and VCOM in each step. Usually just
    /// three or two steps are used.
    ///
    /// When we talk about a "WW" transition or "WB" transition, what we
    /// mean is the difference between the pixel value set in the *last*
    /// display update, and the pixel value of the *current* display update.
    /// So if in the previous update a pixel was white, and later the pixel
    /// turns black, then it's a WB transition and will be handled by the
    /// WB LUT.
    ///
    /// VCOM table is different and explained later, but for the first four
    /// tables, this is how to interpret them. For instance the
    /// lookup for WW in the second row (step 1) could be set to:
    ///
    /// 0x60, 0x02, 0x02, 0x00, 0x00, 0x01 -> last byte = repeat count
    ///  \     |      |    |     |
    ///   \    +------+----+-----+-> number of frames
    ///    \_ four transitions
    ///
    /// The first byte must be read as four two bits integers:
    ///
    /// 0x60 is: 01|10|00|00
    ///
    /// Where each 2 bit number menas:
    /// 00 - Put to ground
    /// 01 - Put to VDH voltage (10v in our config): pixel becomes black
    /// 10 - Put to VDL voltage (-10v in our config): pixel becomes white
    /// 11 - Floating / Not used.
    ///
    /// Then the next four bytes in the row mean how many
    /// "frames" we hold a given state (the frame duration depends on the
    /// frequency set in the PLL, here we configure it to 100 HZ so 10ms).
    ///
    /// So in the above case: hold pixel at VDH for 2 frames, then
    /// hold at VDL for 2 frame. The last two entries say 0 frames,
    /// so they are not used. The final byte in the row, 0x01, means
    /// that this sequence must be repeated just once. If it was 2
    /// the whole sequence would repeat 2 times and so forth.
    ///
    /// The VCOM table is similar, but the bits meaning is different:
    /// 00 - Put VCOM to VCOM_DC voltage
    /// 01 - Put VCOM to VDH+VCOM_DC voltage (see PWR register config)
    /// 10 - Put VCOM to VDL+VCOM_DC voltage
    /// 11 - Floating / Not used.
    ///
    /// The VCOM table has two additional bytes at the end.
    /// The meaning of these bytes apparently is the following (but I'm not
    /// really sure what they mean):
    ///
    /// First additional byte: ST_XON, if (1<<step) bit is set, for
    /// that step all gates are on. Second byte: ST_CHV. Like ST_XON
    /// but if (1<<step) bit is set, VCOM voltage is set to high for this step.
    ///
    /// However they are set to 0 in all the LUTs I saw, so they are generally
    /// not used and we don't use it either.
    pub fn set_waveform_lut(&mut self, speed: i32, no_flickering: bool) {
        if speed > 6 {
            panic!("Speed must be set between 1 and 6");
        }
        let speed = if speed < 1 { 1 } else { speed };

        // Those periods are powers of two so that each successive 'speed'
        // value cuts them in half cleanly.
        let period = 64; // Num. of frames for single direction change.
        let hperiod = period / 2; // Num. of frames for back-and-forth change.

        // core doesn't have pow, do a simple looping lambda version
        let pow = |mut x: i32, v| {
            if v == 0 {
                1
            } else {
                for _ in 0..=v {
                    x = x * x;
                }
                if v > 0 {
                    x
                } else {
                    -x
                }
            }
        };

        // Actual period is scaled by the speed factor
        let period: u8 = core::cmp::max(period / pow(2, speed - 1), 1)
            .try_into()
            .unwrap();
        let hperiod: u8 = core::cmp::max(hperiod / pow(2, speed - 1), 1)
            .try_into()
            .unwrap();
        if speed <= 3 && !no_flickering {
            // For low speed everything is charge-neutral, even WB/BW.

            // Phase 1: long go-inverted-color.
            self.set_lut_row(Lut::vc, 0, 0b00_000000, [period, 0, 0, 0], 2);
            self.set_lut_row(Lut::bw, 0, 0b01_000000, [period, 0, 0, 0], 2);
            self.set_lut_row(Lut::wb, 0, 0b10_000000, [period, 0, 0, 0], 2);

            // Phase 2: short ping/pong.
            self.set_lut_row(Lut::vc, 1, 0b00_00_0000, [hperiod, hperiod, 0, 0], 2);
            self.set_lut_row(Lut::bw, 1, 0b10_01_0000, [hperiod, hperiod, 0, 0], 1);
            self.set_lut_row(Lut::wb, 1, 0b01_10_0000, [hperiod, hperiod, 0, 0], 1);

            // Phase 3: long go-target-color.
            self.set_lut_row(Lut::vc, 2, 0b00_000000, [period, 0, 0, 0], 2);
            self.set_lut_row(Lut::bw, 2, 0b10_000000, [period, 0, 0, 0], 2);
            self.set_lut_row(Lut::wb, 2, 0b01_000000, [period, 0, 0, 0], 2);
            // For this speed, we use the same LUTs for WW/BB as well. We
            // will clear it for no flickering modes.
            self.ww.lut = self.bw.lut;
            self.bb.lut = self.wb.lut;
        } else {
            // Speed > 3
            // For greater than 3 we use non charge-neutral LUTs for WB/BW
            // since the inpulse is short and it gets reversed when the
            // pixel changes color, so that's not a problem for the display,
            // however we still need to use charge-neutral LUTs for WW/BB.

            // Phase 1 for BW/WB. Just go to target color.
            // Phase 1 for WW/BB. Invert, go back.
            let p = period;
            self.set_lut_row(Lut::vc, 0, 0b00_00_00_00, [p * 4, 0, 0, 0], 1);
            self.set_lut_row(Lut::bw, 0, 0b10_00_00_00, [p * 4, 0, 0, 0], 1);
            self.set_lut_row(Lut::wb, 0, 0b01_00_00_00, [p * 4, 0, 0, 0], 1);
            self.set_lut_row(Lut::ww, 0, 0b01_10_00_00, [p * 2, p * 2, 0, 0], 1);
            self.set_lut_row(Lut::bb, 0, 0b10_01_00_00, [p * 2, p * 2, 0, 0], 1);

            // If no flickering mode is enabled, we use an empty
            // waveform BB and WW. The screen will be fully refreshed every
            // self.full_update_period updates.
            //
            // !!! WARNING !!!
            //
            // For BB/WW, to just re-affirm the pixel color applying only the
            // voltage needed for the target color will result in microparticles
            // to be semi-permanently polarized towards one way, with damages
            // that often go away in one day or alike, but it may ruin the
            // display forever insisting enough. So we just put the pixels to
            // ground, and from time to time do a full refresh.
            if no_flickering {
                self.ww.lut.fill(0);
                self.bb.lut.fill(0);
            }
        }
    }
}

impl fmt::Display for VComLut {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self.lut)
        // write!(f, "{}", self.lut.iter().map(f).collect())
    }
}
impl fmt::Display for TransitionLut {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self.lut)
    }
}

impl fmt::Display for CustomLut {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "vcom: {}\nww: {}\nbw: {}\nwb: {}\nbb: {}\n",
            self.vcom, self.ww, self.bw, self.wb, self.bb
        )
    }
}
