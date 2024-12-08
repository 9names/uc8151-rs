use core::{any, ops::Index};

use crate::lut::{TransitionLut, VComLut};

pub fn set_pixels_for_greyscale(
    grey: &mut [u8],
    fb1: &mut [u8],
    fb2: &mut [u8],
    shift: u8,
    level: u8,
) -> bool {
    let mut has_changed = false;
    let fb_size = crate::FRAME_BUFFER_SIZE;
    let pixel_count = crate::HEIGHT * crate::WIDTH;
    fb1.fill(0);
    fb2.fill(0);
    let level2 = level + 1;
    let level3 = level + 2;

    for i in 0..pixel_count as usize {
        let byte = i >> 3;
        let bit = 1 << (7 - (i & 7));
        let converted = (255 - grey[i]) >> shift;

        has_changed = has_changed
            || if converted == level {
                true
            } else if converted == level + 1 {
                fb1[byte] |= bit;
                fb2[byte] |= bit;
                true
            } else if converted == level + 2 {
                fb1[byte] |= bit;
                true
            } else {
                fb2[byte] |= bit;
                false
            };
    }

    has_changed
}

/// Load and render the greyscale image specified. The
/// image format must be: 4 bytes WWHH width,height
/// unsigned 16 bit, big endian. Followed by width*height
/// bytes. Each byte is a pixel with color 0 (black) to
/// 255 (white).
pub fn load_greyscale_image(file: &[u8], greyscale: Option<u8>) {
    let greyscale = greyscale.unwrap_or(16);
    //update_greyscale
}

/// Update the display in greyscale "faked mode" using the image
/// into the framebuffer "buffer". The buffer should be width*height
/// pixels (depending on the display size) bytes. Each byte has
/// a value in the range 0-255, from black to white.
pub fn update_greyscale(greyscale: u8) {
    let greyscales = [32, 16, 8, 4]; // Must be a power of two
    assert!(greyscales.contains(&greyscale));
    // Frames needed to go from white to black, using  too large a
    // number may damager the display, but using a bit larger number
    // may improve contrast
    let frames_to_black = 32;

    // todo: validate this
    let shift = greyscales.partition_point(|&x| x == greyscale);

    let orig_speed = 0; // Todo: self.speed
    let mut fb: [u8; crate::FRAME_BUFFER_SIZE as usize] = [0; crate::FRAME_BUFFER_SIZE as usize];

    let mut fb1: [u8; crate::FRAME_BUFFER_SIZE as usize] = [0; crate::FRAME_BUFFER_SIZE as usize];
    let mut fb2: [u8; crate::FRAME_BUFFER_SIZE as usize] = [0; crate::FRAME_BUFFER_SIZE as usize];

    let mut lut = TransitionLut { lut: [0u8; 42] };
    let mut vcom_lut = VComLut { lut: [0u8; 44] };
    let mut anypixel = false;

    for g in (0..greyscale).step_by(3) {
        let any_pixel = set_pixels_for_greyscale(&mut fb, &mut fb1, &mut fb2, shift as u8, g + 1);
        if any_pixel {
            // Transfer the "old" image, so that for difference
            // with the new we transfer via .update() we create
            // the four set of conditions (WW, BB, WB, BW) based
            // on the difference between the bits in the two
            // images.

            // TODO: access to hardware
            // self.send_image(fb2, old=True)

            // We set the framebuffer with just the pixels of the level
            // of grey we are handling in this cycle, so now we apply
            // the voltage for a time proportional to this level (see
            // the setting of LUT[1], that is the number of frames).
            lut.lut[0] = 0x55; // Go black
            lut.lut[5] = 1; // Repeat 1 for all
            lut.lut[1] = (frames_to_black / greyscale - 1) * (g + 1);
            // self.write(CMD_LUT_WW,LUT)
            lut.lut[1] = (frames_to_black / greyscale - 1) * (g + 2);
            // self.write(CMD_LUT_BB,LUT)
            lut.lut[1] = (frames_to_black / greyscale - 1) * (g + 3);
            // self.write(CMD_LUT_WB,LUT)

            // These pixels will be unaffected, none of them is
            // of the three colors handled in this cycle
            lut.lut[1] = 0;
            lut.lut[5] = 0;

            // self.write(CMD_LUT_BW,LUT)

            // Minimal VCOM LUT to avoid any unneeded wait
            lut.lut[0] = 0; // Already zero, just to make it obvious
            lut.lut[5] = 1;
            lut.lut[1] = (frames_to_black / greyscale - 1) * (g + 3);

            // Finally update
            // self.update(blocking=True)
        }
        // Restore a normal LUT based on configured speed
        // self.set_speed(orig_speed,no_flickering=orig_no_flickering)
        // self.wait_and_switch_off()
    }
}
