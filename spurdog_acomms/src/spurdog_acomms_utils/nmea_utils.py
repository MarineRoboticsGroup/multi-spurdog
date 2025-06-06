import numpy as np
import rospy
from datetime import datetime

def parse_nmea_sentence(nmea_sentence):
    # Get the NMEA data type and the data
    # Convert nmea string to a comma separated list
    nmea_stripped = nmea_sentence.split("*")[0] # Remove checksum
    nmea_string = nmea_stripped.split(",") # split into fields
    nmea_type = nmea_string[0] # Get the NMEA string type
    data = nmea_string[1:] # Get the NMEA data in list form
    return nmea_type, data

def parse_nmea_cacmd(nmea_data: list):
    """ Modem-to-host acknowledgement of a ping command, via the PingRequest service
        $CACMD,PNG,SRC,DEST,RATE,CDR,NumDatabytes*CS
        $CACMD,PNG,0,1,1,0,3*22
    """
    # Get the fields
    if len(nmea_data) != 6:
        rospy.logerr("[%s] CACMD data length is not 6, instead %s!" % (rospy.Time.now(), len(nmea_data)))
        return None
    else:
        src = int(nmea_data[1])
        dest = int(nmea_data[2])
        return src, dest

def parse_nmea_cacma(nmea_data: list):
    """ Modem-to-host acknowledgement of a recieved ping, sent via the PingRequest service
        $CACMA,TIME,PNG,SRC,DEST,<>,<>,NumDatabytes,<>*CS
        $CACMA,2025-03-17T17:16:29.639994,PNG,0,1,56.7,6.62,3,0,0*74
    """
    # Get the fields
    if len(nmea_data) != 9:
        rospy.logerr("[%s] CAMCA data length is not 9, instead %s!" % (rospy.Time.now(), len(nmea_data)))
        return None
    else:
        recieved_ping_time = rospy.Time.from_sec(datetime.strptime(nmea_data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp())
        src = int(nmea_data[2])
        dest = int(nmea_data[3])
        return src, dest, recieved_ping_time

def parse_nmea_cacmr(nmea_data: list):
    """ Modem-to-host acknowledgement of a completed ping command, via the PingRequest service
        $CACMR,PNR,TIME,Orig DEST,Orig SRC,OWTT(sec),
        $CACMR,PNR,2025-05-29T15:09:05.461473,1,0,0.001997,6.3,7.65,1,0,*41
    """
    # Get the fields
    if len(nmea_data) != 10:
        rospy.logerr("[%s] CACMR data length is not 10, instead %s! %s" % (rospy.Time.now(), len(nmea_data), nmea_data))
        return None
    else:
        # Remove any items that contain ''
        nmea_data = [item for item in nmea_data if item != '']
        # Remove the last item if it is empty
        recieved_ping_time = rospy.Time.from_sec(datetime.strptime(nmea_data[1],"%Y-%m-%dT%H:%M:%S.%f").timestamp())
        src = int(nmea_data[3])
        dest = int(nmea_data[2])
        return src, dest, recieved_ping_time

def parse_nmea_carfp(nmea_data: list):
    """ Modem-to-host acknowledgement that a minipacket has been recieved
        $CARFP,TIME,<>,SRC,DEST,<>,<>,<>,<>,Data,<>,<>*CS
        Data: <>;NumFrames;HeaderData(in Hex),<>;NumFrames;MSGData(in Hex)
        $CARFP,2025-05-29T15:26:07.286444,0,1,0,1,-1,0,0,1;6;0801000700ff,,*01
        Successful ping generates: $CARFP,2025-06-05T17:35:06.389465,0,1,0,1,-1,0,0,1;6;0801000700ff,,*00

    """
    # Get the fields
    if len(nmea_data) != 11:
        rospy.logerr("[%s] CARFP data length is not 11, instead %s! %s" % (rospy.Time.now(), len(nmea_data),nmea_data))
        return None, None, None, None, None
    else:
        # Remove any items that contain ''
        nmea_data = [item for item in nmea_data if item != '']
        # Get the time, src and dest
        recieved_ping_time = rospy.Time.from_sec(datetime.strptime(nmea_data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp())
        src = int(nmea_data[2])
        dest = int(nmea_data[3])
        if nmea_data[5] != "-1": # $CARFP data[5]=-1 indicates a ping minipacket
            num_frames = (len(nmea_data[8].split(";"))/3)-1
            hex_data = nmea_data[8].split(";")[-1] # Splits the data into frames, then takes the last frame (avoiding header data)
            hex_payload = bytearray.fromhex(hex_data)
            data_payload = hex_payload.decode("utf-8") if hex_payload else ""
        elif nmea_data[5] != "5": # $CARFP data[5]=5 indicates a ros minipacket
            # Get the number of frames and the header data
            num_frames = (len(nmea_data[8].split(";"))/3)-1
            data_payload = ""
        else:
            rospy.logerr("[%s] Unexpected CARFP data: %s!" % (rospy.Time.now(), nmea_data))
            return None, None, None, None, None
        return src, dest, recieved_ping_time, num_frames, data_payload

def parse_nmea_carev(nmea_data: list):
    "Modem to host software version message"
    if nmea_data[1] == "AUV":
        firmware_version = nmea_data[2].split(".")
    else:
        return None
    return firmware_version