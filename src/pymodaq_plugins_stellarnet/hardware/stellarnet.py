# 
# Copyright 2012-2013 Jeremy Hall
# 
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
# 
#      http://www.apache.org/licenses/LICENSE-2.0
# 
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
#

### edits December 2019 ECN
#   edited to make Python 3 compatible
#   1) 'xrange' depricated in Python 3, replaced with 'range'
#   2) 'range' in Python 2, replaced with 'list(range(...))' to create list objects
#   3) 'print' updated to 'print()'
#   4) 'payload = bytearray(line[9:9 + count*2].decode("hex"))'
#      updated for python3 to the following line
#      'payload = bytearray.fromhex(line[9:9 + count*2])'
###
"""
stellarnet - StellarNet USB spectrometer support.

Devices use Cyprus Semiconductor CY7C68013A microcontroller (EZ-USB) which 
initially enumerates as idVendor/idProduct = 0x04B4/0x8613. Documentation for 
these devices can be found here: http://www.cypress.com/?rID=38801.

Installation:

This module requires Python 2.7 and depends on pyusb 1.0.x. One way to install 
it is using pip:

Install pip: http://www.pip-installer.org/en/latest/installing.html
Install pyusb: pip install pyusb==1.0.0a3

After plugging in a device, you should be able to test the basic operation of 
the driver:

    sudo python stellarnet.py info

Access to USB devices on Linux systems is restricted and requires root access. 
However, it is possible to overcome this restriction by installing the custom 
udev rules file provided: 99-local.rules. Install this file as 
/etc/udev/rules.d/99-local.rules and reboot the system to provide non-root
access to StellarNet devices.
"""

__author__ = 'Jeremy Hall'

import usb.core
import usb.util
import time
import argparse
import sys
import struct
import re
import os

class StellarNetError(Exception):
    """Base class for StellarNet errors."""

    def __init__(self, message):
        Exception.__init__(self, message)

class NotFoundError(StellarNetError):
    """Raised when USB device cannot be found."""

    pass

class ArgumentError(StellarNetError):
    """Raised when argument in error."""

    pass

class ArgTypeError(ArgumentError):
    """Raised when argument type is incorrect."""

    pass

class ArgRangeError(ArgumentError):
    """Raised when argument is out of range."""

    pass

class TimeoutError(StellarNetError):
    """Raised when device operation times out."""

    pass

class StellarNet(object):
    """
    Represents a StellarNet spectrometer.
    """

    DEVICE_ID_ADDR = 0x20
    """The address of the stored string containing device identification."""
    
    COEFF_C1_ADDR = 0x80
    """The address of the string containing coefficient C1"""
    
    COEFF_C2_ADDR = 0xA0
    """The address of the string containing coefficient C2"""
    
    COEFF_C3_ADDR = 0xC0
    """The address of the string containing coefficient C3"""
    
    COEFF_C4_ADDR = 0xE0
    """The address of the string containing coefficient C4"""

    # Ids of the default USB device (Cyprus Semiconductor/EZ-USB)
    _DEFAULT_VENDOR_ID = 0x04B4
    _DEFAULT_PRODUCT_ID = 0x8613

    # Ids of the ReNumerated USB device (StellarNet/USB2EPP)
    _STELLARNET_VENDOR_ID = 0x0BD7
    _STELLARNET_PRODUCT_ID = 0xA012

    # Device to host control request
    _IN_DEVICE = usb.util.CTRL_IN | \
        usb.util.CTRL_TYPE_VENDOR | \
        usb.util.CTRL_RECIPIENT_DEVICE

    # Host to device control request
    _OUT_DEVICE = usb.util.CTRL_OUT | \
        usb.util.CTRL_TYPE_VENDOR | \
        usb.util.CTRL_RECIPIENT_DEVICE
        
    # Maps detector type to number of pixels
    _PIXEL_MAP = {
        1:2048, # CCD - 2048
        2:1024, # CCD - 1024
        3:2048, # PDA - 2048
        4:1024, # PDA - 1024
        5:512,  # InGaAs - 512
        6:1024  # InGaAs - 1024
                # PDA - 3600 (don't know the detector type for this one)
    }
    
    # Maps smoothing parameter to sliding window size
    _WINDOW_MAP = {0:0, 1:5, 2:9, 3:17, 4:33}
    
    # Next auto-assigned id
    _next_auto_id = 0
    
    def __init__(self, device):
        """Constructor. Prepare device for use."""
        
        self._device = device
        self._init_config()
        
    def __del__(self):
        """Destructor. Release device resources."""

        if self._device:
            usb.util.dispose_resources(self._device)
            self._device = None

    def set_config(self, **kwargs):
        """
        Sets the device configuration.
        
        :param int_time: (optional) Integer; the integration time in milliseconds.
        :param x_timing: (optional) Integer; the XTiming rate.
        :param x_smooth: (optional) Integer; the boxcar smoothing window size.
        :param scans_to_avg: (optional) Integer; the # of scans to be averaged together.
        :param temp_comp: (optional) Integer; temperature compensation (not implemented).
        """
        
        update_int_time = False
        for key in kwargs:
            value = kwargs[key]
            if key == 'int_time':
                if value not in range(2, 65536):
                    raise ArgRangeError(key)
                update_int_time = True
            elif key == 'x_timing':
                if value not in range(1, 4):
                    raise ArgRangeError(key)
                update_int_time = True
            elif key == 'x_smooth':
                if value not in StellarNet._WINDOW_MAP:
                    raise ArgRangeError(key)
            elif key == 'scans_to_avg':
                if value < 1:
                    raise ArgRangeError(key)
            elif key == 'temp_comp':
                if value != 0:
                    raise ArgRangeError(key)
            else:
                # Ignore unknown keys
                continue
            
            self._config[key] = value

        if update_int_time:
            self._set_device_timing()
            
    def get_config(self):
        """Gets the device configuration. Returns a dictionary."""
        
        return self._config.copy()
    
    def get_device_id(self):
        """Gets the device id. Returns a string."""
        
        return self._config['device_id']

    def read_spectrum(self):
        """Reads and returns a spectrum from the spectrometer. Returns an array 
        of short integers.
        
        See :class:`StellarNet`.set_config() for a description of the parameters 
        that control the operation of the spectrometer or the post-processing of 
        the spectrum.
        """
        
        data = self._smooth_data(self._read_data())
        scans_to_avg = self._config['scans_to_avg']
        if scans_to_avg > 1:
            summed = list(data)
            for i in range(2, scans_to_avg + 1):
                data = self._smooth_data(self._read_data())
                for j in range(len(summed)):
                    summed[j] = int(summed[j]*((i - 1)/float(i)) + data[j]/float(i))
            data = summed
            
        return data
    
    def set_stored_bytes(self, address, data):
        """Set stored bytes.
        
        :param address: Integer; the address of the string to set.
        :param data: String; the string value to be set
        """

        if address not in list(range(0x00, 0x100, 0x20)):
            raise ArgRangeError('address')

        if len(data) != 0x20:
            raise ArgRangeError('data')

        write_flag = 0x45
        payload = [0, address, write_flag]
        payload.extend(bytearray(data))
        self._device.ctrl_transfer(StellarNet._OUT_DEVICE, 0xB6, 0, 0, payload)

    def get_stored_bytes(self, address):
        """Get stored bytes. Returns bytearray.
        
        :param address: Integer; the address of the string to get.
        """

        if address not in list(range(0x00, 0x100, 0x20)):
            raise ArgRangeError('address')

        payload = [0, address, 0]
        self._device.ctrl_transfer(StellarNet._OUT_DEVICE, 0xB6, 0, 0, payload)

        # The first byte of the data returned from the device is a copy of the
        # command request byte, in this case 0xB5. Since this never useful it is
        # excluded from the value returned by this method
        return self._device.ctrl_transfer(
            StellarNet._IN_DEVICE, 0xB5, 0, 0, 0x21)[1:]

    def get_stored_string(self, address):
        """Get stored bytes. Returns string.
        
        :param address: Integer; the address of the string to get.
        """

        return ''.join(map(chr, self.get_stored_bytes(address)))
    
    def compute_lambda(self, pixel):
        """Compute lambda from the pixel index. Returns the pixel's wavelength (float).
        
        :param pixel: Integer; the pixel index on which to perform the computation.
        """

        if not isinstance(pixel, int):
            raise ArgTypeError('pixel')
        
        pixels = StellarNet._PIXEL_MAP[self._config['det_type']]
        if pixel < 0 or pixel >= pixels:
            raise ArgRangeError('pixel')

        if 'coeffs' not in self._config:
            raise ArgumentError('Device has no stored coefficients')
        
        coeffs = self._config['coeffs']
        return ((pixel**3)*coeffs[3]/8.0 + 
                (pixel**2)*coeffs[1]/4.0 + 
                pixel*coeffs[0]/2.0 + 
                coeffs[2])

    def print_info(self):
        """Print device information."""

        print('--- Device Information')
        print(('idVendor:      {0:04X}'.format(self._device.idVendor)))
        print(('idProduct:     {0:04X}'.format(self._device.idProduct)))
        print(("iManufacturer: '{0}'".format(
                usb.util.get_string(self._device, 100, self._device.iManufacturer))))
        print(("iProduct:      '{0}'".format(
                usb.util.get_string(self._device, 100, self._device.iProduct))))
        print('--- Stored Strings:')
        for address in range(0x00, 0x100, 0x20):
            print((r"{0:02X} '{1}'".format(address, self.get_stored_string(address))))

    def _init_config(self):
        # Set default configuration
        self._config = dict()
        self.set_config(int_time = 100,
                        x_timing = 3,
                        x_smooth = 0,
                        scans_to_avg = 1,
                        temp_comp = 0)

        try:
            # Try to fetch and parse lambda coefficients
            coeffs = [float(self.get_stored_string(addr).split()[0])
                for addr in [StellarNet.COEFF_C1_ADDR, StellarNet.COEFF_C2_ADDR, 
                             StellarNet.COEFF_C3_ADDR, StellarNet.COEFF_C4_ADDR]]
        except:
            # Assume its an older device that doesn't store information
            self._config['det_type'] = 1
        else:
            self._config['coeffs'] = coeffs
            
            # Fetch and parse detector type
            det_type = self.get_stored_bytes(StellarNet.COEFF_C1_ADDR)[31] - ord('0')
            if det_type not in StellarNet._PIXEL_MAP:
                raise ArgRangeError('det_type')
            self._config['det_type'] = det_type
            
            # Assign model and device id
            p = re.compile('(.+) #(\d+)')
            m = p.match(self.get_stored_string(StellarNet.DEVICE_ID_ADDR))
            if m:
                self._config['model'] = m.group(1)
                self._config['device_id'] = m.group(2)
                return
        
        # In the absence of device-specific information use:
        if self._device.bus and self._device.address:
            # bus and address
            auto_id = '{:x}{:x}'.format(self._device.bus, self._device.address)
        else:
            # next id
            auto_id = str(StellarNet._next_auto_id)
            StellarNet._next_auto_id += 1
        
        self._config['model'] = 'unknown'
        self._config['device_id'] = 'auto_id_' + auto_id

    def _set_device_timing(self):
        """Send device timing information to the device."""
        
        int_time = self._config['int_time']
        msb = int_time>>8
        lsb = int_time&0xFF
        xt = 1<<(self._config['x_timing'] + 1)
        msd = 0x1F if int_time > 1 else 0x0F

        self._device.ctrl_transfer(StellarNet._OUT_DEVICE, 0xB4, 0, 0, [msb, lsb, xt, msd])

    def _read_data(self):
        """Read data from the device."""

        # Start the integration
        self._device.ctrl_transfer(StellarNet._OUT_DEVICE, 0xB2, 0, 0)

        # Set the timeout to be 500ms longer than the expected integration time
        int_time = self._config['int_time']
        timeout = (int_time + 500)/1000.0

        # Sleep for 90% of the integration time to avoid busy waiting
        # for the result and causing unnecessary usb traffic
        start = time.time()
        time.sleep(int_time*.0009)
        
        while True:
            # Check if data is available
            reponse = self._device.ctrl_transfer(StellarNet._IN_DEVICE, 0xB3, 0, 0, 2)

            # If device is busy wait for next check else if device is ready wait
            # before reading data. Note: reading data without waiting occasionally
            # results in a right-shifted spectrum which becomes more frequent with
            # shorter integration times..
            time.sleep(.003)
            if reponse[1]:
                break
            
            if time.time() - start > timeout:
                raise TimeoutError('Read spectrum')

        # Read the CCD data and convert it to an array of 
        # 16-bit integers from the little-endian data buffer
        pixels = StellarNet._PIXEL_MAP[self._config['det_type']]
        return struct.unpack_from('<{}H'.format(pixels),
            self._device.read(usb.util.ENDPOINT_IN | 8, pixels*2))
    
    def _smooth_data(self, src):
        """Apply boxcar smoothing to data."""
        
        win_span = StellarNet._WINDOW_MAP[self._config['x_smooth']]
        if win_span == 0:
            return src

        # Smooth middle, start indexes are inclusive, limit indexes are exclusive
        pixels = StellarNet._PIXEL_MAP[self._config['det_type']]
        dst = [0]*len(src)
        half_span = win_span/2
        src_start = half_span
        src_limit = pixels - half_span
        win_start = 0
        win_limit = win_span
        win_sum = sum(src[win_start:win_limit])
        dst[src_start] = win_sum/win_span
        for i in range(src_start + 1, src_limit):
            win_sum += src[win_limit] - src[win_start]
            dst[i] = win_sum/win_span
            win_start += 1
            win_limit += 1
        
        # Smooth left end
        src_start = 0
        win_sum = src[src_start]
        dst[src_start] = src[src_start]
        j = src_start + 1
        for i in range(j, half_span):
            win_sum += src[j + 0] + src[j + 1]
            j += 2
            dst[i] = win_sum/j
            
        # Smooth right end
        src_start = pixels - 1
        win_sum = src[src_start]
        dst[src_start] = src[src_start]
        j = src_start - 1
        for i in range(j, pixels - half_span - 1, -1):
            win_sum += src[j - 0] + src[j - 1]
            j -= 2
            dst[i] = win_sum/(src_start - j)
        
        return dst
    
def _load_firmware(device, filename):
    """Load firmware into device from Intel-hex-formatted file."""

    # CPU control and status register address
    CPUCS = 0xE600

    # Put CPU into reset
    device.ctrl_transfer(StellarNet._OUT_DEVICE, 0xA0, CPUCS, 0, [0x01])

    # Load firmware
    __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__)))
    with open(os.path.join(__location__, filename), 'r') as fh:
        for line in fh:
            count = int(line[1:1 + 2], 16)
            if count:
                address = int(line[3:3 + 4], 16)
                #payload = bytearray(line[9:9 + count*2].decode("hex"))
                # python3 needs the line below
                payload = bytearray.fromhex(line[9:9 + count*2])
                device.ctrl_transfer(
                    StellarNet._OUT_DEVICE, 0xA0, address, 0, payload)

    # Take CPU out of reset
    device.ctrl_transfer(StellarNet._OUT_DEVICE, 0xA0, CPUCS, 0, [0x00])

def _set_usb_config(device):
    """Set USB configuration"""
    
    value = -1
    try:
        value = device.get_active_configuration().bConfigurationValue
    except usb.core.USBError:
        pass

    if value != 1:
        device.set_configuration()

def find_devices():
    """
    Find all USB-connected StellarNet devices.
    
    This function returns a tuple of StellarNet objects or raises NotFoundError 
    if no devices are found.
    """
    
    devices = usb.core.find(find_all=True,
        idVendor=StellarNet._STELLARNET_VENDOR_ID,
        idProduct=StellarNet._STELLARNET_PRODUCT_ID)
    renumerated_count = len(list(devices))

    devices = usb.core.find(find_all=True,
        idVendor=StellarNet._DEFAULT_VENDOR_ID,
        idProduct=StellarNet._DEFAULT_PRODUCT_ID)
    default_count = len(list(devices))
    
    total_count = renumerated_count + default_count
    if total_count == 0:
        raise NotFoundError('No devices found')
    
    # ReNumerate all default devices
    for device in devices:
        _set_usb_config(device)
        _load_firmware(device, 'stellarnet.hex')
        
        # We will not need the default devices again so release their resources
        usb.util.dispose_resources(device)
        
    # Find all the ReNumerated devices
    start = time.time()
    while True:
        time.sleep(.25)

        devices = usb.core.find(find_all=True,
            idVendor=StellarNet._STELLARNET_VENDOR_ID,
            idProduct=StellarNet._STELLARNET_PRODUCT_ID)
            
        if len(devices) >= total_count:
            break
        
        if time.time() - start > 5:
            raise NotFoundError('ReNumerated device(s)')
    
    # Select the default configuration for all devices
    for device in devices:
        _set_usb_config(device)
    
    return tuple(StellarNet(device) for device in devices)

_param_keys = ['int_time', 'x_timing', 'x_smooth', 'scans_to_avg', 'temp_comp']

def _get_params(args):
    params = vars(args)
    return {k:params[k] for k in _param_keys if params[k]}

def select_device(args, return_all=False):
    try:
        devices = find_devices()
    except NotFoundError:
        print('No devices found')
        sys.exit(0)
        
    if return_all:
        return devices

    if args.device:
        for device in devices:
            if device.get_device_id() == args.device:
                return device
        print('The selected device cannot be found')
        sys.exit(0)

    if len(devices) > 1:
        print('Multiple devices, select one with -d option:')
        print(('  {}'.format(', '.join([d.get_device_id() for d in devices]))))
        sys.exit(0)

    return devices[0]

def _print_info(args):
    """Print information about the device."""

    if args.list:
        print('Available devices:')
        devices = select_device(args, True)
        print(('  {}'.format(', '.join([d.get_device_id() for d in devices]))))
    else:
        select_device(args).print_info()
    
def _plot_spectrum(args):
    """Read the device and show a simple ascii plot of the data."""

    device = select_device(args)
    params = _get_params(args)
    device.set_config(**params)
    pixels = device.read_spectrum()
    pixel_count = len(pixels)
    min_value = min(pixels)
    max_value = max(pixels)

    cols = 80
    rows = int(cols*0.5)
    table = [[' ' for x in range(cols)] for y in range(rows)]

    for i in range(pixel_count):
        x = int(cols*i/float(pixel_count))
        y = int(rows*(pixels[i] - min_value)/float(max_value - min_value + 1))
        table[y][x] = '*'

    try:
        x_label = '{}-{} nm'.format(
            int(round(device.compute_lambda(0))),
            int(round(device.compute_lambda(pixel_count - 1))))
    except  ArgumentError:
        x_label = '0-{} px'.format(pixel_count - 1)
    print(('Plot: x-axis: {}, y-axis: {}-{} counts'.format(
            x_label, min_value, max_value)))

    for x in reversed(list(range(rows))):
        for y in range(cols):
            sys.stdout.write(table[x][y])
        sys.stdout.write('\n')

class Timer:    
    def __enter__(self):
        self.start = time.time()
        return self

    def __exit__(self, *args):
        self.end = time.time()
        self.interval = self.end - self.start

def _run_perf_test(args):
    """Run performance test."""

    device = select_device(args)
    params = _get_params(args)
    device.set_config(**params)
    
    with Timer() as t:
        for _ in range(args.repeats):
            device.read_spectrum()
    
    print('Run parameters:')
    params = device.get_config()
    for k in _param_keys:
        print(("  {:<12} : {}".format(k, params[k])))
    print(('Repeats      {}'.format(args.repeats)))
    print(('Total time   {:.3f} secs'.format(t.interval)))
    print(('Average time {:.3f} secs'.format(t.interval/args.repeats)))
    
def _print_data(args):
    """Print scan data."""
    
    device = select_device(args)
    params = _get_params(args)
    device.set_config(**params)
    
    print(('{{"data":[{}],"timestamp":{}}}'.format(
        ','.join(map(str, device.read_spectrum())), 
        int(time.time()*1000))))
                    
def main(argv=None):
    """Excerise StellarNet spectrometer driver."""

    if argv is None:
        argv = sys.argv[1:]

    parser = argparse.ArgumentParser(description='Exercise StellarNet spectrometer.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-d', '--device', help='select device')
    subparsers = parser.add_subparsers()

    param_parser = argparse.ArgumentParser(description='spectrometer parameters', add_help=False)
    param_parser.add_argument('-i', '--int_time', type=int, help='integration time (ms)')
    param_parser.add_argument('-t', '--x_timing', type=int, help='x-timing rate')
    param_parser.add_argument('-s', '--x_smooth', type=int, help='noise smoothing')
    param_parser.add_argument('-a', '--scans_to_avg', type=int, help='scan averaging')
    param_parser.add_argument('-p', '--temp_comp', type=int, help='temperature compensation (not implemented)')

    info_parser = subparsers.add_parser('info', help='print device information (default)')
    info_parser.add_argument('-l', '--list', action='store_true', help='list available devices')
    info_parser.set_defaults(func=_print_info)
   
    plot_parser = subparsers.add_parser('plot', parents=[param_parser], help='plot spectrum')
    plot_parser.set_defaults(func=_plot_spectrum)

    perf_parser = subparsers.add_parser('perf', parents=[param_parser], help='run performance test')
    perf_parser.add_argument('-r', '--repeats', type=int, default=1, help='scan repetition count')
    perf_parser.set_defaults(func=_run_perf_test)
    
    data_parser = subparsers.add_parser('data', parents=[param_parser], help='print json-formatted scan data')
    data_parser.set_defaults(func=_print_data)
    
    args = parser.parse_args()
    args.func(args)

if __name__ == '__main__':
    sys.exit(main())

