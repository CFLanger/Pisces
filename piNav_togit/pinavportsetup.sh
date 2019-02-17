#!/bin/bash

echo -e "\$PMTK251,115200*1F" >/dev/ttyS0 
stty -F /dev/ttyS0 115200

#// ----- set the gps fix/update rate to 1 Hz: 
#echo -e "\$PMTK220,1000*1F" >/dev/ttyS0  
#// ----- set the gps fix/update rate to 1.5 Hz: 
#echo -e "\$PMTK220,666*28" >/dev/ttyS0 
#// ----- set the gps fix/update rate to 2 Hz: 
echo -e "\$PMTK220,500*2B" >/dev/ttyS0 
#// ----- set the gps fix/update rate to 2.857 Hz:
#echo -e "\$PMTK220,350*28" >/dev/ttyS0
#// ----- set the gps fix/update rate to 5 Hz: 
#echo -e "\$PMTK220,200*2C" >/dev/ttyS0
#// ----- set the gps fix/update rate to 10 Hz:
#echo -e "\$PMTK220,100*2F" >/dev/ttyS0

# GPRMC, GPGGA and GSA intervals only 
#echo -e "\$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" >/dev/ttyS0
# GPRMC and GPGGA intervals only
echo -e "\$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28" >/dev/ttyS0
# GPRMC interval only
#echo -e "\$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" >/dev/ttyS0

#// --- to get WAAS (Wide Area Augmentation System):
#// PMTK_API_SET_DGPS_MODE to enable WAAS as DGPS Source (not RTCM) 
echo -e "\$PMTK301,2*2E" >/dev/ttyS0

#// also need to enable SBAS (Satellite-based Augmentation System):
#// PMTK_API_SET_SBAS_ENABLED to enable search for SBAS satellite
echo -e "\$PMTK313,1*2E" >/dev/ttyS0

#// really have no idea what this does or if it is necessary:
#// PMTK_API_SET_SBAS_Mode to integrity mode
echo -e "\$PMTK319,1*24" >/dev/ttyS0

#//----- disable nav speed threshold:
#//PMTK_SET_Nav Speed threshold (set to 0 to disable)
echo -e "\$PMTK386,0*23" >/dev/ttyS0  

#// ----- Active Interference Cancellation (AIC):
#// PMTK_CMD_AIC_MODE set to enabled
echo -e "\$PMTK286,1*23" >/dev/ttyS0
