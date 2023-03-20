from collections import namedtuple

Packer = namedtuple('Packer', ['format', 'length'])
Attribute = namedtuple('Attribute', ['name', 'attribute_type'])
PackHelp = namedtuple('PackHelp',
                      ['start', 'format', 'length', 'end', 'ordinal'])


class Packing(object):
    def __init__(self):
        self.map = dict()

        self.map['uint8'] = Packer('B', 1)
        self.map['uint16'] = Packer('H', 2)
        self.map['int16'] = Packer('h', 2)
        self.map['uint32'] = Packer('L', 4)
        self.map['int32'] = Packer('l', 4)
        self.map['uint64'] = Packer('Q', 8)
        self.map['bool'] = Packer('?', 1)
        self.map['float'] = Packer('f', 4)
        self.map['6s'] = Packer('6s', 6)

    def get_packing(self, type: str) -> Packer:
        try:
            return self.map[type]

        except KeyError:
            raise Exception(f"type {type} not defined in Packing")


class SettingsIndex(object):
    def pack_help(self, name: str) -> PackHelp:
        """
        Given the name of a Settings attribute, return a PackHelp
        namedtuple which contains:
            - start index for that attributes
            - struct.pack format character for it
            - how many bytes it will use
            - end index for it
            - where it is in the list of settings
        """

        byte_position = 0
        for ordinal in range(len(self.attributes)):
            attribute_type = self.attributes[ordinal].attribute_type
            packing = self._packing.get_packing(attribute_type)

            if self.attributes[ordinal].name != name:
                byte_position += packing.length
                continue

            else:
                return PackHelp(
                    start=byte_position,
                    format=packing.format,
                    length=packing.length,
                    end=byte_position+packing.length,
                    ordinal=ordinal
                )

        raise Exception(f"{name} not in settings")

    def __init__(self):

        self._packing = Packing()

        self.attributes = list()
        self.attributes.append(Attribute('status_flags', 'uint8'))  # 0
        self.attributes.append(Attribute('status_output', 'uint8'))

        self.attributes.append(Attribute('uart_main_baud', 'uint8'))
        self.attributes.append(Attribute('uart_aux_baud', 'uint8'))

        # TODO: May be a bug in either the blueprint docs or the firmware
        self.attributes.append(Attribute('net_mac_addr', '6s'))

        self.attributes.append(Attribute('net_ip_addr', 'uint32'))
        self.attributes.append(Attribute('net_ip_subnet', 'uint32'))
        self.attributes.append(Attribute('net_ip_gateway', 'uint32'))
        self.attributes.append(Attribute('net_ip_dns', 'uint32'))
        self.attributes.append(Attribute('net_tcp_port', 'uint16'))

        self.attributes.append(Attribute('env_flags', 'uint8'))  # 10
        self.attributes.append(Attribute('env_pressure_ofs', 'int32'))
        self.attributes.append(Attribute('env_salinity', 'uint16'))
        self.attributes.append(Attribute('env_vos', 'uint16'))

        self.attributes.append(Attribute('ahrs_flags', 'uint8'))
        self.attributes.append(Attribute('acc_min_x', 'int16'))
        self.attributes.append(Attribute('acc_min_y', 'int16'))
        self.attributes.append(Attribute('acc_min_z', 'int16'))
        self.attributes.append(Attribute('acc_max_x', 'int16'))
        self.attributes.append(Attribute('acc_max_y', 'int16'))
        self.attributes.append(Attribute('acc_max_z', 'int16'))  # 20

        self.attributes.append(Attribute('mag_valid', 'bool'))
        self.attributes.append(Attribute('mag_hard_x', 'float'))
        self.attributes.append(Attribute('mag_hard_y', 'float'))
        self.attributes.append(Attribute('mag_hard_z', 'float'))
        self.attributes.append(Attribute('mag_soft_x', 'float'))
        self.attributes.append(Attribute('mag_soft_y', 'float'))
        self.attributes.append(Attribute('mag_soft_z', 'float'))
        self.attributes.append(Attribute('mag_field', 'float'))
        self.attributes.append(Attribute('mag_error', 'float'))

        self.attributes.append(Attribute('gyro_offset_x', 'int16'))  # 30
        self.attributes.append(Attribute('gyro_offset_y', 'int16'))
        self.attributes.append(Attribute('gyro_offset_z', 'int16'))

        self.attributes.append(Attribute('ahrs_yaw_ofs', 'uint16'))
        self.attributes.append(Attribute('ahrs_pitch_ofs', 'uint16'))
        self.attributes.append(Attribute('ahrs_roll_ofs', 'uint16'))

        self.attributes.append(Attribute('xcvr_flags', 'uint8'))
        self.attributes.append(Attribute('xcvr_beacon_id', 'uint8'))
        self.attributes.append(Attribute('xcvr_range_tmo', 'uint16'))
        self.attributes.append(Attribute('xcvr_resp_time', 'uint16'))
        self.attributes.append(Attribute('xcvr_yaw', 'uint16'))  # 40
        self.attributes.append(Attribute('xcvr_pitch', 'uint16'))
        self.attributes.append(Attribute('xcvr_roll', 'uint16'))
        self.attributes.append(Attribute('xcvr_posflt_vel', 'uint8'))
        self.attributes.append(Attribute('xcvr_posflt_ang', 'uint8'))
        self.attributes.append(Attribute('xcvr_posflt_tmo', 'uint8'))  # 45
