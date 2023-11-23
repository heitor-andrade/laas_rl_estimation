'''tflayols thomasfla
CNRS 2022
Read a stream of bytes from FTDI usb to serial chipset and parse it acording to this protocol:

'ab x1 x2 ... xn CRC8'

 * 'ab' is a 16bits sync code (0x61 0x62)
 * CRC8 is a Maxim8bits CRC covering 'ab...xn'
 * data_names is an array describing the name given at data 'x1' x2' ...
 * data_scales is an array of scalling to apply to x1 x2...
 * data_format is a string describing the format structure of x1 ... x2 see https://docs.python.org/3/library/struct.html

example:
    data_names = ['time','position','velocity']  
    data_scales = [1e-3, 1, 1]
    data_format = "Iff" # Usigned int16, float, float
'''

import numpy as np
from IPython import embed
import matplotlib.pyplot as plt
''' This will dump data from the ftdi serial port'''


def dump(url='ftdi://ftdi:232h:FT42C48K/1',
         baudrate=12000000,
         nb_bytes=10000000):
    # Enable pyserial extensions
    import pyftdi.serialext
    '''
    ftdi_urls.py 
    Available interfaces:
    ftdi://ftdi:232h:FT42C48K/1   (C232HD-DDHSP-0)
    '''
    # Open a serial port on FTDI device
    port = pyftdi.serialext.serial_for_url(url, baudrate)

    # Receive bytes
    port.flushInput()
    data = port.read(nb_bytes)
    port.close()
    return (data)


'''This will parse data according to a given format, and return a dictionary of numpy arrays'''


def parseData(rawData,
              data_names,
              data_format,
              data_scales,
              print_status=True):
    import struct
    import crcmod.predefined
    crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')
    fmt = "<cc" + data_format + "B"
    frame_size = struct.calcsize(fmt)
    length = len(rawData)
    N = int(length / frame_size)
    times = np.empty([N]) * np.nan
    data = {}
    for key in data_names:
        data[key] = np.empty([N]) * np.nan
    data['crc'] = np.empty([N]) * np.nan
    j = 0
    for i in range(length - frame_size):
        sample = struct.unpack_from(fmt, rawData, i)
        if sample[0:2] == (b'a', b'b'):
            times[j] = sample[2]
            validCRC = (sample[-1] == crc8(rawData[i:i + frame_size - 1]))
            data['crc'][j] = validCRC
            if sample[-1] == crc8(rawData[i:i + frame_size - 1]):
                for n in range(len(data_names)):
                    data[data_names[n]][j] = sample[2 + n] * data_scales[n]
            j += 1
            i += frame_size
            if (print_status and j % 10000 == 0):
                print(f"{int(100*i/length)} %")
        if (j >= N): break
        if (i >= length): break
    return (data)

# Example of use
dataRaw = dump()

data_names = ['time', 'pos_m1', 'pos_m2', 'pos_as5047u', 'vel_as5047u']
data_scales = [25e-6, 1.0, 1.0, 1.0, 1.0]
data_format = "Iffff"

data = parseData(dataRaw, data_names, data_format, data_scales)
plt.plot(data['crc'])
plt.plot(data['pos_m1'])
plt.plot(data['pos_m2'])
plt.plot(data['pos_as5047u'])
plt.plot(data['vel_as5047u'])
plt.ylim(-10,10)
plt.show()
