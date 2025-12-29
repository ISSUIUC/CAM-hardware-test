# What is this file?

This file a place where I put functions that are currently not needed but I already wrote code for.


# tvp5151.h
```cpp
// NOT NEEDED FOR NOW (UNTESTED 12.29.2025): 
bool set_avid_output_enable(bool enable);
// NOT NEEDED FOR NOW (UNTESTED 12.29.2025): 
bool set_avid_out_active_during_vblk(bool active);
```

#  tvp5151.cpp

```cpp
/*
3.21.4 Miscellaneous Controls Register | Bit 2 (Page 31)
HSYNC, VSYNC/PALI, active video indicator (AVID), and FID/GLCO output enables
0 = HSYNC, VSYNC/PALI, AVID, and FID/GLCO are high-impedance (default).
1 = HSYNC, VSYNC/PALI, AVID, and FID/GLCO are active.
Note: This control bit has no effect on the FID/GLCO output when it is programmed to output the
GLCO signal (see bit 3 of address 0Fh). When the GLCO signal is selected, the FID/GLCO output is
always active.

For actual cropping, refer to section 3.13 on page 16. I don't think we need this for Bt.656.

3.21.6 Miscellaneous Output Controls Register | Bit 1 (Page 34)
AVID/CLK_IN function select
0 = CLK_IN (default)
1 = AVID

This function changes two registers' bits' values.
*/
bool tvp5151::set_avid_output_enable(bool enable)
{
    if (!modify_register_bit(TVP_REG_MISC_OUTPUT_CONTROLS, 0x02, enable))
        return false;

    return modify_register_bit(TVP_REG_MISC_CONTROLS, 0x04, enable);
}

/*
3.21.17 Active Video Cropping Start Pixel LSB Register | Bit 2 (Page 41)
AVID active
0 = AVID out active in VBLK (default)
1 = AVID out inactive in VBLK
Active video cropping start pixel LSB [1:0]: The TVP5151 decoder updates the AVID start values only
when this register is written to.

This setting controls how the AVID signal behaves during the Vertical Blanking Interval (VBLK). The Vertical Blanking Interval is the "dead time" between frames where no viewable video is sent.

    0 = AVID out active in VBLK (default): In this mode, the AVID signal continues to pulse high for every line, even during the vertical blanking period. This is often used if downstream hardware needs a consistent timing reference for every single line of the signal, regardless of whether it contains "real" video.

    1 = AVID out inactive in VBLK: In this mode, the AVID signal is forced to logic low (inactive) for the entire duration of the vertical blanking interval. It only begins pulsing again when the first line of "active" video begins. This is very useful for processors because it tells the hardware, "Stop looking for data until the vertical blanking is over."
*/
bool tvp5151::set_avid_out_active_during_vblk(bool active)
{
    return modify_register_bit(TVP_REG_AVID_CROP_START_LSB, 0x04, !active);
}
```