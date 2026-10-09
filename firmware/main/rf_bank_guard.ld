/* DAC ownership changes apply to the whole 128 KiB RF bank. */
ASSERT(_heap_start <= 0x3fca0000, "Static data overlaps the RF DAC bank; reduce IRAM/BSS usage")
