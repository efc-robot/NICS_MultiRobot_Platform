import uuid
def get_mac_address(): 
    mac=uuid.UUID(int = uuid.getnode()).hex[-12:] 
    return ":".join([mac[e:e+2] for e in range(0,11,2)])

def mac2id(mac):
    mac2id_table = {}
    mac2id_table['e4:5f:01:00:d4:20'] = 'AKM_1'
    mac2id_table['e4:5f:01:00:d3:d8'] = 'AKM_2'
    mac2id_table['e4:5f:01:00:d4:5c'] = 'AKM_3'
    mac2id_table['e4:5f:01:1b:a2:39'] = 'AKM_4'
    mac2id_table['e4:5f:01:1b:9e:ca'] = 'AKM_5'
    mac2id_table['e0:d5:5e:26:58:85'] = 'host_0'
    
    if mac in mac2id_table.keys():
        return mac2id_table[mac]
    else:
        return 'dummy'

if __name__ == '__main__':
    print(mac2id(get_mac_address()))
    