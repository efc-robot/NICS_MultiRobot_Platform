import uuid
import os
def get_mac_address(): 
    mac=uuid.UUID(int = uuid.getnode()).hex[-12:] 
    return ":".join([mac[e:e+2] for e in range(0,11,2)])

def mac2id(mac):
    mac2id_table = {'e4:5f:01:00:d4:20':'1',
                    'e4:5f:01:00:d3:d8':'2',
                    'e4:5f:01:00:d4:5c':'3',
                    'e4:5f:01:00:d3:ae':'4'}
    assert mac in mac2id_table.keys()
    return mac2id_table[mac]


if __name__ == '__main__':
    print('car_'+mac2id(get_mac_address()))
    