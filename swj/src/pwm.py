

# b1 b1 00 0f 00 00 period pulse duration crc 1b 1b
# period, pulse, duration 补成4位16进制，crc后两位16进制
# frequency in Hz, 
def get_crc(frequency, duty_cycle, duration):
    period = (int)(1/frequency*1000000)
    pulse = (int)(period * duty_cycle)
    crc = 15+period+pulse+duration
    print("b1 b1 00 0f 00 00"+' %04x'%period+' %04x'%pulse+' %04x'%duration+' %02x'%(crc%256)+" 1b 1b")


get_crc(100, 0.2, 4*16*16*16)