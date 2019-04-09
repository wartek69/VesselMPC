from collections import defaultdict
from copy import copy

LON = 0
LAT = 1
SOG = 2
HDT = 3
COG = 4
EIROT = 5
ReqROT = 6
GPROT = 7
file = 'VL20190305.txt'

def convert_to_csv():
    autopilot_active = False
    f = open(file)
    data = defaultdict(lambda: [0] * 8)
    lastTime = 0
    for line in f.readlines():
        if len(line) > 0 and line[0] == '#':
            continue

        line = line.strip().split()
        time = line[2]
        if time != lastTime and lastTime != 0 and autopilot_active is True:
            data[time] = copy(data[lastTime])

        nmea = line[4]
        if len(nmea) > 3 and nmea[-3] == '*':
            nmeaCode = nmea[3:6]

            if nmeaCode == 'TPS' and nmea.split(',')[1] == '2':
                autopilot_active = True
            elif nmeaCode == 'TPS' and nmea.split(',')[1] != '2':
                autopilot_active = False

            if autopilot_active is True:
                if nmeaCode == 'HDT':
                    data[time][HDT] = nmea.split(',')[1]
                elif nmeaCode == 'ROT':
                    if nmea[1:3] == 'EI':
                        data[time][EIROT] = nmea.split(',')[1]
                    elif nmea[1:3] == 'GP':
                        data[time][GPROT] = nmea.split(',')[1]
                elif nmeaCode == 'RMC':
                    sog = nmea.split(',')[7]
                    status = nmea.split(',')[2]
                    if status == 'V' or sog == '':
                        pass
                    else:
                        cog = nmea.split(',')[8]
                        lat = nmea.split(',')[3]
                        lon = nmea.split(',')[5]

                        data[time][SOG] = sog
                        data[time][COG] = cog
                        data[time][LON] = float(lon[:3]) + (float(lon[3:]) / 60)
                        data[time][LAT] = float(lat[:2]) + (float(lat[2:]) / 60)
                elif nmeaCode == 'TPS' and nmea.split(',')[1] == '2':
                    rrot = nmea.split(',')[4]
                    data[time][ReqROT] = rrot
                lastTime = time
    f.close()

    f = open('output/' + file + 'outall', 'w+')
    f.write('#Time,Lon,Lat,Speed,Heading,Course,EIROT,GPROT,ReqROT\n')
    print('Time,Lon,Lat,Speed,Heading,Course,EIROT,GPROT,ReqROT\n')
    for time in data:
        print('{},{:.7f},{:.7f},{},{},{},{},{},{}'.format(
            time,
            data[time][LON],
            data[time][LAT],
            data[time][SOG],
            data[time][HDT],
            data[time][COG],
            data[time][EIROT],
            data[time][GPROT],
            data[time][ReqROT]
        ))
        f.write('{},{:.7f},{:.7f},{},{},{},{},{},{}\n'.format(
            time,
            data[time][LON],
            data[time][LAT],
            data[time][SOG],
            data[time][HDT],
            data[time][COG],
            data[time][EIROT],
            data[time][GPROT],
            data[time][ReqROT]
        ))
    f.close()

def process_data():
    f = open('output/' + file + 'outall', 'r')
    f2 = open('output/' + file + '.processed', 'w')
    f2.write('#,Time,Lon,Lat,Speed,Heading,Course,EIROT,GPROT,ReqROT\n')

    for line in f.readlines():
        original_line = line
        line = line.strip().split(',')
        if line[3] == 'Speed' or float(line[3]) < 0.2:
            continue
        print(original_line)
        f2.write(original_line)
    f.close()
    f2.close()

# prepares data for mlp
# take note that all the angles are scaled between 0-360
def prepare_data_blackbox():
    f = open('output/' + file + '.processed', 'r')
    f2 = open('output/' + file + '.blackbox.prepared', 'w')
    f2.write("#rot;heading;rrot;speed;lat_rel;lon_rel;heading';rot'\n")
    prev_lat = None
    prev_lon = None
    lines = f.readlines()

    for index, line in enumerate(lines):
        if len(line) > 0 and line[0] == '#':
            continue
        line = line.strip().split(',')
        if prev_lat is None:
            prev_lat = float(line[2])
        if prev_lon is None:
            prev_lon = float(line[1])
        rot = float(line[6]) % 360
        heading = float(line[4]) % 360
        rrot = float(line[8]) % 360
        speed = float(line[3])

        lat_rel = float(line[2]) - prev_lat
        lon_rel = float(line[1]) - prev_lon
        if index + 1 == len(lines):
            i = index - 1
        else:
            i = index
        temp = float(lines[i + 1].strip().split(',')[4])
        next_heading = float(lines[i + 1].strip().split(',')[4]) % 360
        next_rot = float(lines[i + 1].strip().split(',')[6]) % 360
        prev_lat = float(line[2])
        prev_lon = float(line[1])

        f2.write('{};{};{};{};{};{};{};{}\n'.format(
            rot,
            heading,
            rrot,
            speed,
            lat_rel,
            lon_rel,
            next_heading,
            next_rot
        ))
    f2.close()
    #todo lon correction! lon = lon/cos(lat)

def prepare_data_rotdot():
    f = open('output/' + file + '.processed', 'r')
    f2 = open('output/' + file + '.rotdot.prepared', 'w')
    f2.write("#rot;rrot;speed;rotdot'\n")

    lines = f.readlines()

    for index, line in enumerate(lines):
        line = line.strip().split(',')
        if len(line) > 0 and line[0] == '#':
            continue
        rot = float(line[6]) % 360
        rrot = float(line[8]) % 360
        speed = float(line[3])

        if index == len(lines) - 1:
            continue
        rotdot = float(lines[index + 1].strip().split(',')[6]) - float(line[6])
        f2.write('{};{};{};{}\n'.format(
            rot,
            rrot,
            speed,
            rotdot
        ))
    f2.close()





if __name__ == '__main__':
    convert_to_csv()
    process_data()
    prepare_data_blackbox()
    prepare_data_rotdot()
