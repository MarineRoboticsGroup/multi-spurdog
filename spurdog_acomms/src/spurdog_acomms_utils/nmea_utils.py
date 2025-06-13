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
        recieved_ping_time = rospy.Time.from_sec(datetime.strptime(nmea_data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp()).to_sec()
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
        recieved_ping_time = rospy.Time.from_sec(datetime.strptime(nmea_data[1],"%Y-%m-%dT%H:%M:%S.%f").timestamp()).to_sec()
        src = int(nmea_data[3])
        dest = int(nmea_data[2])
        owtt = float(nmea_data[4]) if nmea_data[4] else 0.0  # OWTT in seconds
        return src, dest, recieved_ping_time, owtt

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
        recieved_ping_time = rospy.Time.from_sec(datetime.strptime(nmea_data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp()).to_sec()
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

def parse_nmea_cacst(nmea_data: list):
    """ Modem-to-host report of recieved acoustic signal
       From Recieved Ping: "$CACST,8,0,20250606230123.348554,06,32767,45,0102,0123,07,00,00,00,00,1,001,000,0,5,1,0,150,26.0,9.48,-100,-6.48,-01,0.01,51,25000,5000,0,-1,-1,00*4A"
       From Recieved Ping: "$CACST,[0]Version#,TOA,<?>,<?>,<?>,[5]<?>,SHF-GAIN,SHF-INPSHIFT,SHF-INSHIFT,SHF-P2BSHIFT,[10]Rate,SRC,DEST,PSKError,PacketType,[15]NframesExp,Nbadframes,SNR-RSS,SNR-IN,SNR-OUT,[20]SymbSNR?,MSEError?,DQF,DOP,StdDevNoise(dB),[25]Carrier,BW,PCM,<?>,<?>,<?>*CS
    """
    # Extratc the time of arrival 20250606230123.348554 and conert it to a rospy.Time object
    toa = rospy.Time.from_sec(datetime.strptime(nmea_data[2],"%Y%m%d%H%M%S.%f").timestamp()).to_sec()
    # Extract the source and destination
    src = int(nmea_data[14])
    dest = int(nmea_data[15])
    # Detect if its PSK or FSK based $CACST (is [10] ==0?)
    if nmea_data[13] == "-1":
        # Error in PSK data
        rospy.logerr("[%s] CACST PSK data error: %s!" % (rospy.Time.now(), nmea_data))
        return
    elif nmea_data[13] == "0":
        # FSK data
        # dqf = int(nmea_data[13])
        # symb_snr = float(nmea_data[20])
        rospy.logerr("[%s] CACST FSK data not implemented yet: %s!" % (rospy.Time.now(), nmea_data))
    else:
        # Analyze the Nframes and report the message type:
        nframes = int(nmea_data[18])
        if nframes == 1:
            # Message is a ping:
            msg_type = "ping"
        else:
            # Message is a ping response:
            msg_type = "data"
        # Extract the SNR values
        snr_rss = float(nmea_data[20])
        snr_in = float(nmea_data[21])
        snr_out = float(nmea_data[22])
        #mse_eq = float(nmea_data[21])
    dop = float(nmea_data[26])
    stddev_noise = float(nmea_data[27])
    # Return the parsed data
    return toa, src, dest, msg_type, nframes, snr_rss, snr_in, snr_out, dop, stddev_noise

def parse_nmea_caxst(nmea_data: list):
    """Modem to host report of a transmitted signal
    From Ping: $CAXST,7,20250606,230103.000935,3,0,200,5000,25000,1,0,1,0,0,1,5,2,25000*53"
    From Pckt: $CAXST,7,20250606,230117.336548,3,0,200,5000,25000,1,0,0,0,0,8,5,28,25000*66"
    $CAXST,Version#,Date(YYYYMMDD),TOT(hhmmss.ssssss),ClockStatus,Mode,ProbeLength,BW,Carrier,<?>,SRC,DEST,ACK,<?>,<?>,NframesSent,Rate,Nbytes,Carrier*CS
    """
    # Parse the time as rostime
    msg_tot = nmea_data[1]+nmea_data[2]
    tot = rospy.Time.from_sec(datetime.strptime(msg_tot,"%Y%m%d%H%M%S.%f").timestamp()).to_sec()
    # Parse the SRC, DEST
    src = int(nmea_data[9])
    dest = int(nmea_data[10])
    if nmea_data[11] == "0":
        msg_type = "ping"
    else:
        msg_type = "data"
    nframes = int(nmea_data[13])
    nbytes = int(nmea_data[15])

    return tot, src, dest, msg_type, nframes, nbytes

def parse_nmea_carev(nmea_data: list):
    "Modem to host software version message"
    if nmea_data[1] == "AUV":
        firmware_version = nmea_data[2].split(".")
    else:
        return None
    return firmware_version

def parse_nmea_catxf(nmea_data: list):
    """Modem to host acknowledgement of a transmitted minipacket
    "$CATXF,2*56"
    "$CATXF,28*6E"
    $CATXF,Nbytes*CS
    """
    # Parse the time as rostime
    nbytes = int(nmea_data[0])
    return nbytes